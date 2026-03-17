import json
from influxdb_client.client.influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from communication.rabbitmq import Rabbitmq
from communication.protocol import MODEL_ROUTING_KEY_STATE
from communication.factory import RabbitMQFactory
from influxdb_client.client.write.point import Point
import threading
from datetime import datetime, timezone

class DBRecorderService:
    def __init__(self):
        self.write_api = None
        self.influx_db_org = None
        self.influxdb_bucket = None
        self.rabbitmq = None

    def read_record_request(self, ch, method, properties, body_json):
        print("RECORDING REQUEST CALLED ------------------------------------------------")
        msg = body_json
        print("STATE MESSAGE WAS:", msg)

        if "q_actual" in msg:
            self.write_state(msg)

        else:
            print("Unknown message format:", msg)

    def write_state(self, msg):
        print("WRITING STATE TO DB =========================================================")
        now = datetime.now(timezone.utc)
        robot_mode = msg["robot_mode"]
        q_actual = msg["q_actual"]
        qd_actual = msg["qd_actual"]
        q_target = msg["q_target"]
        timestamp = msg["timestamp"]
        joint_max_speed = msg["joint_max_speed"]
        joint_max_acceleration = msg["joint_max_acceleration"]
        tcp_pose = msg["tcp_pose"]

        point = Point("robotarm_state") \
            .field("robot_mode", robot_mode) \
            .field("q_actual_0", float(q_actual[0])) \
            .field("q_actual_1", float(q_actual[1])) \
            .field("q_actual_2", float(q_actual[2])) \
            .field("q_actual_3", float(q_actual[3])) \
            .field("q_actual_4", float(q_actual[4])) \
            .field("q_actual_5", float(q_actual[5])) \
            .field("qd_actual_0", float(qd_actual[0])) \
            .field("qd_actual_1", float(qd_actual[1])) \
            .field("qd_actual_2", float(qd_actual[2])) \
            .field("qd_actual_3", float(qd_actual[3])) \
            .field("qd_actual_4", float(qd_actual[4])) \
            .field("qd_actual_5", float(qd_actual[5])) \
            .field("q_target_0", float(q_target[0])) \
            .field("q_target_1", float(q_target[1])) \
            .field("q_target_2", float(q_target[2])) \
            .field("q_target_3", float(q_target[3])) \
            .field("q_target_4", float(q_target[4])) \
            .field("q_target_5", float(q_target[5])) \
            .field("timestamp_robot_model", float(timestamp)) \
            .field("joint_max_speed", float(joint_max_speed)) \
            .field("joint_max_acceleration", float(joint_max_acceleration)) \
            .field("tcp_pose_x", float(tcp_pose[0])) \
            .field("tcp_pose_y", float(tcp_pose[1])) \
            .field("tcp_pose_z", float(tcp_pose[2])) \
            .time(now)
                                
       
        self.write_api.write(self.influxdb_bucket, self.influx_db_org, point)

    def write_control(self, msg):
        raise NotImplementedError

    def setup(self, influxdb_config):
        print("SETUP ################# SETUP########")
        print("INFLUX CONFIG:", influxdb_config)
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self.rabbitmq.connect_to_server()

        client = InfluxDBClient(**influxdb_config)
        write_api = client.write_api(write_options=SYNCHRONOUS)
        self.write_api = write_api
        self.influx_db_org = influxdb_config["org"]
        self.influxdb_bucket = influxdb_config["bucket"]
        print("ORG:", self.influx_db_org)
        print("BUCKET:", self.influxdb_bucket)

        self.rabbitmq.subscribe(routing_key=MODEL_ROUTING_KEY_STATE, # TODO: Change to ROUTING_KEY_RECORDER to catch all robotarm. See communication/protocol.py for details
                        on_message_callback=self.read_record_request)

    def start_recording(self):
        try:
            def run():
                self.rabbitmq.start_consuming()

            self.thread = threading.Thread(target=run, daemon=False)
            self.thread.start()
        except KeyboardInterrupt:
            self.rabbitmq.close()