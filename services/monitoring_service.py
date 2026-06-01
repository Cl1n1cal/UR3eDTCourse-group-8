from influxdb_client.client.influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from communication.protocol import ROUTING_KEY_MONITORING, MonitoringMsgKeys
from communication.factory import RabbitMQFactory
from startup.utils.logging_config import create_service_logger
import time
from datetime import datetime, timezone
import utils.monitors as monitors
from typing import Dict, List, Tuple, Union, cast
RobustnessVerdict = Tuple[float, Union[bool, float, Tuple[float, float]]]

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
        self.robustness_results: Dict[str, List[RobustnessVerdict]] = {}

        # Other
        self.time_stamp = time.time()
      
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
        self.monitor_window_steps = monitoring_config.get("monitor_window_steps", 1)
        self.monitor_window_seconds = self.monitor_window_steps * self.step_period
        self.mockup_max_latency = monitoring_config["mockup_max_latency"]
        self.sim_max_latency = monitoring_config["sim_max_latency"]
        self.stuck_joint_q_difference_threshold = monitoring_config["stuck_joint_q_difference_threshold"]
        self.stuck_joint_qd_threshold = monitoring_config["stuck_joint_qd_threshold"]
        self.max_q_error = monitoring_config["max_q_error"]
        self.wear_threshold = float(monitoring_config.get("wear_threshold", 0.5))
        self.wait_time = monitoring_config["wait_time"]
        self.joint_rotation_threshold = float(monitoring_config.get("joint_rotation_threshold", 0.0))

        self.monitors = [monitors.LatencyMonitor(self.sim_max_latency, self.sample_delay, "simulation"),
                        monitors.LatencyMonitor(self.mockup_max_latency, self.sample_delay, "mockup"),
                        monitors.VelocityMonitor(self.mockup_max_velocity, window_seconds=self.monitor_window_seconds),
                        monitors.AccelerationMonitor(self.mockup_max_acceleration, window_seconds=self.monitor_window_seconds),
                        monitors.MismatchMonitor("q", self.max_q_error, window_seconds=self.monitor_window_seconds),
                        monitors.MismatchMonitor("tcp", self.max_q_error, window_seconds=self.monitor_window_seconds),
                        monitors.WearPredictionMonitor(self.wear_threshold, window_seconds=self.monitor_window_seconds),
                        monitors.StuckJointMonitor(joint_index=0, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold, window_seconds=self.monitor_window_seconds),
                        monitors.StuckJointMonitor(joint_index=1, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold, window_seconds=self.monitor_window_seconds),
                        monitors.StuckJointMonitor(joint_index=2, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold, window_seconds=self.monitor_window_seconds),
                        monitors.StuckJointMonitor(joint_index=3, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold, window_seconds=self.monitor_window_seconds),
                        monitors.StuckJointMonitor(joint_index=4, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold, window_seconds=self.monitor_window_seconds),
                        monitors.StuckJointMonitor(joint_index=5, qd_threshold=self.stuck_joint_qd_threshold, q_diff_threshold=self.stuck_joint_q_difference_threshold, window_seconds=self.monitor_window_seconds),
                        monitors.JointRotationMonitor(joint_index=0, rotation_threshold=self.joint_rotation_threshold),
                        monitors.JointRotationMonitor(joint_index=1, rotation_threshold=self.joint_rotation_threshold),
                        monitors.JointRotationMonitor(joint_index=2, rotation_threshold=self.joint_rotation_threshold),
                        monitors.JointRotationMonitor(joint_index=3, rotation_threshold=self.joint_rotation_threshold),
                        monitors.JointRotationMonitor(joint_index=4, rotation_threshold=self.joint_rotation_threshold),
                        monitors.JointRotationMonitor(joint_index=5, rotation_threshold=self.joint_rotation_threshold)
                        ]
        
        self._l.info("Monitoring service initialized with monitors:")
        for monitor in self.monitors:
            self._l.info(f" - {monitor.type}:")
            self._l.info(f"   {monitor.formula}")

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

            if mockup_data is None or sim_data is None:
                self._l.debug("Waiting for both mockup and simulation data to become available...")
                continue

            if self.old_mockup_data is not None and mockup_data.get("time_stamp") == self.old_mockup_data.get("time_stamp"):
                continue

            # Extract last record for latency checks
            time_stamp = time.time()
            wear_data = results.get("wear_prediction")
            for monitor in self.monitors:
                # pass wear_data as a fourth argument; monitors that don't use it will ignore it
                self.robustness_results[monitor.type] = monitor.compute_robustness(mockup_data, sim_data, time_stamp, wear_data)
            
            for monitor, robustness in self.robustness_results.items():
                self._l.debug(f"Monitor {monitor} robustness: {robustness}")

            # Set this after doing the computations
            # Used in the acceleration calculation where we need the difference in velocity and time
            self.old_mockup_data = mockup_data

            rdata = self.create_monitoring_recorder_msg()
            mdata = self.create_monitoring_msg()

            self.record_message(rdata)
            self.rabbitmq.send_message(ROUTING_KEY_MONITORING, mdata)

    def get_historical_data(self):
        query = f'''
        from(bucket: "{self.influxdb_bucket}")
        |> range(start: -{self.wait_time}s, stop: -{self.sample_delay}ms)
        |> filter(fn: (r) => r._measurement == "mockup_state" or r._measurement == "simulation_state" or r._measurement == "wear_prediction")
        |> filter(fn: (r) => contains(value: r._field, set: [
            "joint_max_acceleration","joint_max_speed",
            "q_actual_0","q_actual_1","q_actual_2","q_actual_3","q_actual_4","q_actual_5",
            "qd_actual_0","qd_actual_1","qd_actual_2","qd_actual_3","qd_actual_4","qd_actual_5",
            "q_target_0", "q_target_1", "q_target_2", "q_target_3", "q_target_4", "q_target_5",
            "tcp_pose_0","tcp_pose_1","tcp_pose_2","tcp_pose_3","tcp_pose_4","tcp_pose_5",
            "robot_mode",
            "predicted_wear_level",
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
        fields7 = ["predicted_wear_level"]

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
                    **{field: record.values.get(field) for field in fields6},
                    **{field: record.values.get(field) for field in fields7}
                }
                data[key] = entry

        self._l.debug(f"Retrieved a historical sample from InfluxDB: {data}.")
        
        return data

    def create_monitoring_msg(self):
        msg = []

        for monitor in self.monitors:
            robustness = self.robustness_results.get(monitor.type)
            if not robustness:
                self._l.debug(f"No robustness value found for monitor {monitor.type}. Skipping message creation for this monitor.")
                continue
            verdict = monitors.UR3eMonitor.latest(robustness)
            if verdict is None:
                self._l.debug(f"No robustness value found for monitor {monitor.type}. Skipping message creation for this monitor.")
                continue
            msg.append({MonitoringMsgKeys.TYPE: monitor.type, MonitoringMsgKeys.ROBUSTNESS_VALUE: verdict})
        
        return msg
    
    def create_monitoring_recorder_msg(self):
        timestamp = datetime.fromtimestamp(time.time(), timezone.utc).isoformat()
        fields = {}

        for monitor in self.monitors:
            robustness = self.robustness_results.get(monitor.type)
            if not robustness:
                self._l.debug(f"No robustness value found for monitor {monitor.type}. Skipping recording for this monitor.")
                continue
            verdict = monitors.UR3eMonitor.latest(robustness)
            if verdict is None:
                self._l.debug(f"No robustness value found for monitor {monitor.type}. Skipping recording for this monitor.")
                continue
            fields[monitor.type] = verdict

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