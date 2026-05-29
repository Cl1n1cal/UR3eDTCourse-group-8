from influxdb_client.client.influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from communication.protocol import ROUTING_KEY_MONITORING, MonitoringMsgKeys
from communication.factory import RabbitMQFactory
from startup.utils.logging_config import create_service_logger
import time
from datetime import datetime, timezone
import utils.monitors as monitors
from utils.monitors import JointRotationMonitor

class MonitoringService:
    def __init__(self, step_period):
        self.write_api = None
        self.influx_db_org = None
        self.influxdb_bucket = None
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self._l = create_service_logger("monitoring_service")
        self.step_period = step_period

        # Monitors
        self.monitors = []

        # Robustness results
        self.robustness_results = {}

        # Other
        self.time_stamp = time.time()
        self.step_period = 0.0 # conf file
      
        self.mockup_max_latency = 0.0 # conf file
        self.sim_max_latency = 0.0 # conf file 
        self.mockup_max_velocity = 0.0 # init value
        self.mockup_max_acceleration = 0.0 # init value
        self.stuck_joint_q_difference_threshold = 0.0 # conf file
        self.max_q_error = 0.0 # conf file
        self.min_q_error = 0.0 # conf file
        self.wait_time = 0.0 # conf file
        self.old_mockup_data = None
        self.joint_rotation_threshold = 0.0 # conf file
      
    def setup(self, monitoring_config):
        self._l.info("Monitoring setup with config ", monitoring_config)
        self.rabbitmq.connect_to_server()

        client = InfluxDBClient(**monitoring_config) 
        self.write_api = client.write_api(write_options=SYNCHRONOUS)
        self.query_api = client.query_api()
        self.influx_db_org = monitoring_config["org"]
        self.influxdb_bucket = monitoring_config["bucket"]

        # Monitor related
        self.sample_delay = monitoring_config["sample_delay"]
        self.mockup_max_latency = monitoring_config["mockup_max_latency"]
        self.sim_max_latency = monitoring_config["sim_max_latency"]
        self.stuck_joint_q_difference_threshold = monitoring_config["stuck_joint_q_difference_threshold"]
        self.stuck_joint_qd_threshold = monitoring_config["stuck_joint_qd_threshold"]
        self.max_q_error = monitoring_config["max_q_error"]
        self.wait_time = monitoring_config["wait_time"]
        self.joint_rotation_threshold = float(monitoring_config.get("joint_rotation_threshold", 0.0))

        self.monitors = [monitors.LatencyMonitor(self.sim_max_latency, self.sample_delay, "simulation"),
                         monitors.LatencyMonitor(self.mockup_max_latency, self.sample_delay, "mockup"),
                         monitors.VelocityMonitor(self.mockup_max_velocity),
                         monitors.AccelerationMonitor(self.mockup_max_acceleration),
                         monitors.MismatchMonitor("q", self.max_q_error),
                         monitors.MismatchMonitor("tcp", self.max_q_error),
                         monitors.StuckJointMonitor(joint_index=0, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold),
                         monitors.StuckJointMonitor(joint_index=1, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold),
                         monitors.StuckJointMonitor(joint_index=2, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold),
                         monitors.StuckJointMonitor(joint_index=3, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold),
                         monitors.StuckJointMonitor(joint_index=4, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold),
                         monitors.StuckJointMonitor(joint_index=5, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold),
                         JointRotationMonitor(joint_index=0, rotation_threshold=self.joint_rotation_threshold),
                         JointRotationMonitor(joint_index=1, rotation_threshold=self.joint_rotation_threshold),
                         JointRotationMonitor(joint_index=2, rotation_threshold=self.joint_rotation_threshold),
                         JointRotationMonitor(joint_index=3, rotation_threshold=self.joint_rotation_threshold),
                         JointRotationMonitor(joint_index=4, rotation_threshold=self.joint_rotation_threshold),
                         JointRotationMonitor(joint_index=5, rotation_threshold=self.joint_rotation_threshold),
                         ]
        
        self._l.info("Monitoring service initialized with monitors:")
        for monitor in self.monitors:
            self._l.info(f" - {monitor.name}:")
            self._l.info(f"   {monitor.monitor}")

    def cleanup(self):
        self.rabbitmq.close()

    def start_monitoring(self):
        if self.rabbitmq == None:
            return
        try:
            self.monitor_loop()
        except KeyboardInterrupt:
            self._l.info("Monitoring service interrupted by user.")
            self.cleanup()
    
    def monitor_loop(self):
        time.sleep(self.wait_time) # Make sure everything is set up before starting to monitor
        self._l.info("Woke up! Starting monitoring loop now.")

        while True:
            time.sleep(self.step_period)
            # Fetch mockup and sim data in parallel
            results = self.get_historical_data()
            mockup_data = results.get("mockup_state")
            sim_data = results.get("simulation_state")

            self._l.debug(f"mockup_data: {mockup_data}")
            self._l.debug(f"sim_data: {sim_data}")

            # Extract last record for latency checks
            time_stamp = time.time()
            for monitor in self.monitors:
                self.robustness_results[monitor.name] = monitor.compute_robustness(mockup_data, sim_data, time_stamp)
            
            for monitor, robustness in self.robustness_results.items():
                self._l.debug(f"Monitor {monitor} robustness: {robustness}")

            # Set this after doing the computaions
            # Used in the acceleration calculation where we need the diffence in velocity and time
            self.old_mockup_data = mockup_data

            rdata = self.create_monitoring_recorder_msg()
            mdata = self.create_monitoring_msg()

            self.record_message(rdata)
            self.rabbitmq.send_message(ROUTING_KEY_MONITORING, mdata)

    def get_historical_data(self):
        query = f'''
        from(bucket: "{self.influxdb_bucket}")
        |> range(start: -{self.wait_time}s, stop: -{self.sample_delay}ms)
        |> filter(fn: (r) => r._measurement == "mockup_state" or r._measurement == "simulation_state")
        |> filter(fn: (r) => contains(value: r._field, set: [
            "joint_max_acceleration","joint_max_speed",
            "q_actual_0","q_actual_1","q_actual_2","q_actual_3","q_actual_4","q_actual_5",
            "qd_actual_0","qd_actual_1","qd_actual_2","qd_actual_3","qd_actual_4","qd_actual_5",
            "q_target_0", "q_target_1", "q_target_2", "q_target_3", "q_target_4", "q_target_5",
            "tcp_pose_0","tcp_pose_1","tcp_pose_2","tcp_pose_3","tcp_pose_4","tcp_pose_5",
            "robot_mode",
        ]))
        |> group(columns: ["_measurement", "_field"])
        |> last()
        |> pivot(rowKey: ["_time", "_measurement"], columnKey: ["_field"], valueColumn: "_value")
        |> sort(columns: ["time_stamp", "measurement"])
        '''
        tables = self.query_api.query(query, org=self.influx_db_org)

        data = {}
        fields = [f"q_actual_{i}" for i in range(6)]
        fields1 = [f"tcp_pose_{i}" for i in range(6)]
        fields2 = [f"qd_actual_{i}" for i in range(6)]
        fields3 = ["robot_mode"]
        fields4 = ["joint_max_speed"]
        fields5 = ["joint_max_acceleration"]
        fields6 = [f"q_target_{i}" for i in range(6)]

        for table in tables:
            for record in table.records:
                key = record.get_measurement()
                entry = {
                    "time_stamp": record.get_time().timestamp(),
                    **{field: record.values.get(field) for field in fields},
                    **{field: record.values.get(field) for field in fields1},
                    **{field: record.values.get(field) for field in fields2},
                    **{field: record.values.get(field) for field in fields3},
                    **{field: record.values.get(field) for field in fields4},
                    **{field: record.values.get(field) for field in fields5},
                    **{field: record.values.get(field) for field in fields6}
                }
                data[key] = entry

        self._l.debug(f"Retrieved a historical sample from InfluxDB: {data}.")
        
        return data

    def create_monitoring_msg(self):
        msg = []

        for monitor in self.monitors:
            robustness = self.robustness_results[monitor.name]
            verdict = monitors.UR3eMonitor.latest(robustness)
            if verdict is None:
                self._l.warning(f"No robustness value found for monitor {monitor.name}. Skipping message creation for this monitor.")
                continue
            msg.append({MonitoringMsgKeys.TYPE: monitor.type, MonitoringMsgKeys.ROBUSTNESS_VALUE: verdict})
        
        return msg
    
    def create_monitoring_recorder_msg(self):
        timestamp = datetime.fromtimestamp(time.time(), timezone.utc).isoformat()
        fields = {}

        for monitor in self.monitors:
            robustness = self.robustness_results[monitor.name]
            verdict = monitors.UR3eMonitor.latest(robustness)
            if verdict is None:
                self._l.warning(f"No robustness value found for monitor {monitor.name}. Skipping recording for this monitor.")
                continue
            fields[monitor.name] = verdict

        return {
            "measurement": "monitoring_robustness",
            "time": timestamp,
            "tags": {"source": "monitor_service"},
            "fields": fields,
        }

    def record_message(self, body_json):
        self._l.debug("New record msg:")
        self._l.debug(body_json)
        try:
            if(self.write_api != None and self.influx_db_org != None and self.influxdb_bucket != None):
                self.write_api.write(self.influxdb_bucket, self.influx_db_org, body_json)
        except Exception as e:
            self._l.warning("Failed to write to InfluxDB")
            self._l.debug("",exc_info=e)
            return