from influxdb_client.client.influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from communication.protocol import ROUTING_KEY_CTRL, CtrlMsgFields, CtrlMsgKeys, RobotArmStateKeys, RobotMode
from communication.factory import RabbitMQFactory
from startup.utils.logging_config import create_service_logger
import numpy as np
import threading
import mstlo_python as mstlo
import time
from datetime import datetime, timezone

class MonitoringService:
    def __init__(self):
        self.write_api = None
        self.influx_db_org = None
        self.influxdb_bucket = None
        self.publisher = RabbitMQFactory.create_rabbitmq()
        self.consumer = RabbitMQFactory.create_rabbitmq()
        self._l = create_service_logger("monitoring_service")
        self.monitor = None
        self.mockup_latency_monitor = None
        self.sim_latency_monitor = None
        self.monitor_latency_monitor = None
        self.monitor_max_latency = 0 # conf file
        self.max_latency = 0 # conf file
        self.stuck_joint_q_difference_threshold = 0 # conf file
        self.max_error = 0.1
        self.min_error = -0.1
        self.spec_formula = "(q_diff < $max_error && q_diff > $min_error)"
        self.mockup_latency_formula = "(time_diff <= $max_latency)"
        self.velocity_formula = "G[0, 4.5](qd_ratio <= 1.0)"
        self.acceleration_formula = "G[0, 4.5](qdd_ratio <= 1.0)"
        self.monitor_latency_formula = "time_diff < $monitor_max_latency"
        self.stuck_joint_formula = "F[0, 4.5]((qd == 0.0) && (q_diff > $stuck_joint_q_difference_threshold))" # Only evaluated if the robot is running

        self._velocity_monitor = None
        self._velocity_spec = None
        self.time_stamp = time.time()
        self.event = threading.Event()

    def setup(self, monitoring_config):
        self._l.info("Monitoring setup with config ", monitoring_config)
        self.publisher.connect_to_server()
        self.consumer.connect_to_server()
        self.consumer.subscribe(routing_key=ROUTING_KEY_CTRL,
                                on_message_callback=self.read_control_message)

        client = InfluxDBClient(**monitoring_config) 
        self.write_api = client.write_api(write_options=SYNCHRONOUS)
        self.query_api = client.query_api()
        self.influx_db_org = monitoring_config["org"]
        self.influxdb_bucket = monitoring_config["bucket"]
        self.sample_delay = monitoring_config["sample_delay"]
        self.max_latency = monitoring_config["max_latency"]
        self.monitor_max_latency = monitoring_config["monitor_max_latency"]
        self.stuck_joint_q_difference_threshold = monitoring_config["stuck_joint_q_difference_threshold"]


    def _initialize_monitor(self):
        spec_vars = mstlo.Variables()
        spec_vars.set("max_error", self.max_error)
        spec_vars.set("min_error", self.min_error)
        spec = mstlo.parse_formula(self.spec_formula)
        self.monitor = mstlo.Monitor(
            formula=spec, semantics="Rosi", variables=spec_vars
        )
        self._l.info(f"Monitoring service initialized with monitor: {self.monitor}")

        # Mockup latency monitor
        mockup_latency_vars = mstlo.Variables()
        mockup_latency_vars.set("max_latency", self.max_latency)
        mockup_latency_spec = mstlo.parse_formula(self.mockup_latency_formula)
        self.mockup_latency_monitor = mstlo.Monitor(
            formula=mockup_latency_spec, semantics="DelayedQuantitative", variables=mockup_latency_vars
        )
        self._l.info(f"Monitoring service initialized with monitor: {self.mockup_latency_monitor}")

        # Monitor service latency monitor
        monitor_latency_vars = mstlo.Variables()
        monitor_latency_vars.set("monitor_max_latency", self.monitor_max_latency)
        monitor_latency_spec = mstlo.parse_formula(self.monitor_latency_formula)
        self.monitor_latency_monitor = mstlo.Monitor(
            formula=monitor_latency_spec, semantics="DelayedQuantitative", variables=monitor_latency_vars
        )
        self._l.info(f"Monitoring service initialized with monitor: {self.monitor_latency_monitor}")

        # Velocity monitor. Uses None for synchronization so we don't interpolate values between the records.
        # We are assuming that records are written often enough and have to input a fake value at time stamp 4.5 so we cannot interpolate
        self.velocity_spec = mstlo.parse_formula(self.velocity_formula)
        self.acceleration_spec = mstlo.parse_formula(self.acceleration_formula)

        stuck_joint_vars = mstlo.Variables()
        stuck_joint_vars.set("stuck_joint_q_difference_threshold", self.stuck_joint_q_difference_threshold)
        self.stuck_joint_spec = mstlo.parse_fomula(self.stuck_joint_formula)

    def read_control_message(self, ch, method, properties, message: dict):
        self._l.info(f"Received control message: {message}")
        msg_type = message.get(CtrlMsgKeys.TYPE)
        self.latest_ctrl_msg = msg_type


    def record_message(self, body_json):
        self._l.debug("New monitoring msg:")
        self._l.debug(body_json)
        try:
            if(self.write_api != None and self.influx_db_org != None and self.influxdb_bucket != None):
                self.write_api.write(self.influxdb_bucket, self.influx_db_org, body_json)
        except Exception as e:
            self._l.warning("Failed to write to InfluxDB")
            self._l.debug("",exc_info=e)
            return

    def cleanup(self):
        self.consumer.close()
        self.publisher.close()

    def start_monitoring(self):
        if self.publisher == None:
            return
        try:
            def run():
                if self.publisher == None:
                    return
                self.publisher.start_consuming()

            self.thread = threading.Thread(target=run, daemon=False)
            self.mon_thread = threading.Thread(target=self.monitor_thread, daemon=False)
            self.mon_thread.start()
            self.thread.start()
        except KeyboardInterrupt:
            self.cleanup()
            

    def process_state_sample(self, ch, method, properties, body_json):
        # get robot arm state history from influx db
        #self.time_stamp = time.time()
        data = self.get_historical_data()
        robustness = self.compute_latency_robustness(data)
        if robustness is None:
            return
        self._l.debug(f"Computed robustness: {robustness}")
        #self.record_message(self.create_robustness_msg(robustness, "q_diff_robustness"))

    def milliseconds_to_seconds(self, milliseconds):
        return milliseconds/1000.0
    
    def monitor_thread(self):
        self._initialize_monitor()
        time.sleep(10) # Let the mockup initialize
        while True:
            time.sleep(self.milliseconds_to_seconds(self.sample_delay)) # time.sleep takes seconds as parameter
            
            # Mockup latency
            time_stamp = time.time()
            last_mockup_data = self.get_last_data("mockup_state")
            mockup_latency_robustness = self.compute_latency_robustness(last_mockup_data, "mockup_state", time_stamp)

            # Sim latency
            time_stamp = time.time()
            last_sim_data = self.get_last_data("simulation_state")
            sim_latency_robustness = self.compute_latency_robustness(last_sim_data, "simulation_state", time_stamp)

            # Mockup velocity
            mockup_data = self.get_historical_data("mockup_state") # Retrieves a list of dicts
            mockup_velocity_robustness = self.compute_mockup_velocity_robustness(mockup_data)

            # Mockup acceleration
            mockup_acceleration_robustness = self.compute_mockup_acceleration_robustness(mockup_data)

            # Mokcup stuck joint detection
            mockup_stuck_joint = self.compute_mockup_stuck_joint(mockup_data)

            # Monitor latency - should be last
            monitor_latency_robustness = self.compute_monitor_latency_robustness()
            

            if mockup_latency_robustness is None or sim_latency_robustness is None:
                self._l.debug(f"Robustness was none")
                continue
            self._l.debug(f"Computed mockup latency robustness: {mockup_latency_robustness}")
            self._l.debug(f"Computed sim latency robustness: {sim_latency_robustness}")
            print("mockup latency robustness:", mockup_latency_robustness)
            print("sim latency robustness", sim_latency_robustness)
            print("mockup velocity robustness", mockup_velocity_robustness)
            print("mockup acceleration robustness", mockup_acceleration_robustness)
            print("monitor latency robustness", monitor_latency_robustness)

            # Update the time stamp - used by the monitor latency monitor
            self.time_stamp = time.time()
    
    def compute_mockup_stuck_joint(self, mockup_data):
        if not mockup_data:
            return None
        
        results = []
        steps = [[] for _ in range(6)]
        for i in range(6):
            for element in mockup_data:
                state = element[RobotArmStateKeys.ROBOT_MODE]
                if state == RobotMode.ROBOT_MODE_RUNNING:
                    qd = element[f"qd_current_{i}"]
                    q_diff = abs(element[f"q_target_{i}"] - element[f"q_actual_{i}"])
                    time = element["time_stamp"]
                    steps[i].append((qd, q_diff, time))
                    # TODO: Work from here





    # Could be changed to query the database for the last time the monitor service wrote to the db
    def compute_monitor_latency_robustness(self):
        tmp_time = time.time()
        time_diff = tmp_time - self.time_stamp

        monitor_latency_robustness = self.monitor_latency_monitor.update(
            signal="time_diff", value=time_diff, timestamp=tmp_time
        )

        return monitor_latency_robustness.verdicts()

    def compute_mockup_velocity_robustness(self, mockup_data):

        steps = []
        for record in mockup_data:
            #if record.get("robot_mode") != "Running":
            #    continue
            qd_ratio = max(abs(record[f"qd_actual_{i}"]) for i in range(6)) / record["joint_max_speed"]
            steps.append((qd_ratio, record["time_stamp"]))

        if not steps:
            return None
        
        # Normalize the time stamps to fit [0, 4.5]
        # The oldest record will be 0 and the latest record will be closest to 4.5
        t0 = steps[0][1]
        steps = [(val, ts - t0) for val, ts in steps]

        # We are going to use he update batch method so we need to make sure that there is a value at
        # time stamp 4.5 or the verdict will return None
        steps.append((steps[-1][0], 4.5))

        batch = {
            "qd_ratio" : steps
        }

        velocity_monitor = mstlo.Monitor(
            formula=self.velocity_spec, semantics="DelayedQuantitative", synchronization="None"
        )
        if velocity_monitor is None:
            raise RuntimeError("Call start_monitoring() before compute_mockup_velocity_robustness")
        self._l.info(f"Monitoring service initialized with monitor: {velocity_monitor}")


        output = velocity_monitor.update_batch(batch)
        verdicts = output.verdicts()
        if not verdicts:
            return None

        return min(val for _, val in verdicts)
    
    def compute_mockup_acceleration_robustness(self, mockup_data):

        steps = []
        for i in range(len(mockup_data)):
            curr = mockup_data[i]
            prev = mockup_data[i-1]

            dt = curr["time_stamp"]- prev["time_stamp"]
            if dt == 0:
                continue
            
            # Of the 6 joints, get the one with the largest acceleration
            acc_max = max(
                abs(curr[f"qd_actual_{j}"] - prev[f"qd_actual_{j}"]) / dt for j in range(6)
            )

            acc_ratio = acc_max / curr["joint_max_acceleration"]
            steps.append((acc_ratio, curr["time_stamp"]))

        if not steps:
            return None
        
        # Normalize the time stamps to fit [0, 4.5]
        # The oldest record will be 0 and the latest record will be closest to 4.5
        t0 = steps[0][1]
        steps = [(val, ts - t0) for val, ts in steps]

        # We are going to use he update batch method so we need to make sure that there is a value at
        # time stamp 4.5 or the verdict will return None
        steps.append((steps[-1][0], 4.5))

        batch = {
            "qdd_ratio" : steps
        }

        acceleration_monitor = mstlo.Monitor(
            formula=self.acceleration_spec, semantics="DelayedQuantitative", synchronization="None"
        )
        if acceleration_monitor is None:
            raise RuntimeError("Call start_monitoring() before compute_mockup_velocity_robustness")
        self._l.info(f"Monitoring service initialized with monitor: {acceleration_monitor}")


        output = acceleration_monitor.update_batch(batch)
        verdicts = output.verdicts()
        if not verdicts:
            return None

        return min(val for _, val in verdicts)



    def get_historical_data(self, measurement: str):
        query = f'''
        from(bucket: "{self.influxdb_bucket}")
        |> range(start: -5s, stop: -{self.sample_delay}ms)
        |> filter(fn: (r) => r._measurement == "{measurement}")
        |> filter(fn: (r) => contains(value: r._field, set: [
            "joint_max_acceleration","joint_max_speed","q_actual_0","q_actual_1","q_actual_2","q_actual_3","q_actual_4","q_actual_5",
            "qd_actual_0","qd_actual_1","qd_actual_2","qd_actual_3","qd_actual_4","qd_actual_5", "q_target_0", "q_target_1", "q_target_2",
            "q_target_3", "q_target_4", "q_target_5",
            "tcp_pose_0","tcp_pose_1","tcp_pose_2","tcp_pose_3","tcp_pose_4","tcp_pose_5",
            "robot_mode" 
        ]))
        |> group(columns: ["_measurement", "_field"])
        |> pivot(rowKey: ["_time", "_measurement"], columnKey: ["_field"], valueColumn: "_value")
        |> sort(columns: ["_time"])
        '''
        tables = self.query_api.query(query, org=self.influx_db_org)

        data = []
        fields = [f"q_actual_{i}" for i in range(6)]
        fields1 = [f"tcp_pose_{i}" for i in range(6)]
        fields2 = [f"qd_actual_{i}" for i in range(6)]
        fields3 = ["robot_mode"]
        fields4 = ["joint_max_speed"]
        fields5 = ["joint_max_acceleration"]
        fields6 = [f"q_target_{i}" for i in range(6)]

        for table in tables:
            for record in table.records:
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
                data.append(entry)

        self._l.debug(f"Retrieved a historical sample from InfluxDB: {data}.")
        
        return data
    
    def get_last_data(self, measurement: str):
        query = f'''
        from(bucket: "{self.influxdb_bucket}")
        |> range(start: -1h, stop: -{self.sample_delay}ms)
        |> filter(fn: (r) => r._measurement == "{measurement}")
        |> filter(fn: (r) => contains(value: r._field, set: [
            "q_actual_0","q_actual_1","q_actual_2","q_actual_3","q_actual_4","q_actual_5",
            "qd_actual_0","qd_actual_1","qd_actual_2","qd_actual_3","qd_actual_4","qd_actual_5",
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

        for table in tables:
            for record in table.records:
                key = record.get_measurement()
                entry = {
                    "time_stamp": record.get_time().timestamp(),
                    **{field: record.values.get(field) for field in fields},
                    **{field: record.values.get(field) for field in fields1},
                    **{field: record.values.get(field) for field in fields2},
                    **{field: record.values.get(field) for field in fields3}
                }
                data[key] = entry

        self._l.debug(f"Retrieved a historical sample from InfluxDB: {data}.")
        
        return data

    def compute_latency_robustness(self, sample_data, measurement: str, time_stamp):
        if self.monitor is None or self.mockup_latency_monitor is None:
            raise RuntimeError("Call start_monitoring() before compute_robutsness")

        """
        required_measurements = ("mockup_state", "simulation_state")
        missing_measurements = [name for name in required_measurements if name not in sample_data]
        if missing_measurements:
            self._l.debug(f"Skipping robustness computation, missing measurements: {missing_measurements}")
            return None


        # calculate difference between mockup and sim values for q_actual
        error = sum((sample_data["mockup_state"][f"q_actual_{i}"] - sample_data["simulation_state"][f"q_actual_{i}"])**2 for i in range(6))**0.5
        self._l.debug(f"Calculated eucledian distance between mockup and sim q_actual values: {error}")
        # get latest time stamp from the sample data
        latest_timestamp = max(sample_data["mockup_state"]["time_stamp"], sample_data["simulation_state"]["time_stamp"])
        robustness = self._monitor.update(
            signal="q_diff", value=error, timestamp=latest_timestamp
        )
        """

        # Calculate the difference in time stamps for the mockup and the monitor service
        #print("sample data", sample_data)
        monitor_adjusted_time = time_stamp - (self.milliseconds_to_seconds(self.sample_delay))
        mockup_time = sample_data[measurement]["time_stamp"]
        time_diff = np.absolute(mockup_time - monitor_adjusted_time)
        print(f"{measurement} time:", monitor_adjusted_time)
        print(f"{measurement} adj time:",  time.ctime(monitor_adjusted_time))
        print(f"{measurement} time:", mockup_time)
        print(f"{measurement} time:", time.ctime(mockup_time))
        print(f"timediff:", time_diff)
        mockup_latency_robustness = self.mockup_latency_monitor.update(
            signal="time_diff", value=time_diff, timestamp=monitor_adjusted_time
        )

        return mockup_latency_robustness.verdicts()
    
    """
    def create_robustness_msg(self, robustness, measurement_name: str):
        # Store the robustness in the influxdb. Duplicate records on the same timestamp will just be updated.
        records = []
        for t, rob in robustness:
            ts = int(t * 1e9)

            # if the rob is inf, set it to a large number
            if rob[0] == float("-inf"):
                rob = (-20.0, rob[1])
            if rob[1] == float("inf"):
                rob = (rob[0], 20.0)

            records.append(
                {
                    "measurement": measurement_name,
                    "tags": {"source": "monitor_service"},
                    "time": ts,
                    "fields": {
                        "robustness_lower_bound": rob[0],
                        "robustness_upper_bound": rob[1],
                    },
                }
            )

        return records
    """