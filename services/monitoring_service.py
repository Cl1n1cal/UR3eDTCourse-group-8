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

        # Monitors
        self.latency_monitor = None
        self.sim_latency_monitor = None
        self.mockup_velocity_monitor = None
        self.mockup_acceleration_monitor = None
        self.mockup_stuck_joint_monitor_0 = None
        self.mockup_stuck_joint_monitor_1 = None
        self.mockup_stuck_joint_monitor_2 = None
        self.mockup_stuck_joint_monitor_3 = None
        self.mockup_stuck_joint_monitor_4 = None
        self.mockup_stuck_joint_monitor_5 = None
        self.sim_vs_mockup_monitor = None
        self.mockup_tcp_monitor = None

        # Formulae
        self.mockup_latency_formula = "(time_diff < $mockup_max_latency)"
        self.sim_latency_formula = "(time_diff < $sim_max_latency)"
        self.mockup_velocity_formula = "(qd <= $mockup_max_velocity)" # Cannot compare two signals so have to use a variable
        self.mockup_acceleration_formula = "(qdd <= mockup_max_acceleration)"
        self.mockup_stuck_joint_formula = "(qd <= $stuck_joint_qd_threshold && q_diff > $stuck_joint_q_difference_threshold && robot_mode >= 1)" # does not have == for the robot_mode
        self.sim_vs_mockup_formula = "(q_diff < $max_q_error && q_diff > $min_q_error)"
        self.mockup_tcp_formula = "(q_diff < $max_q_error && q_diff > $min_q_error)"

        # Other
        self.time_stamp = time.time()
        self.event = threading.Event()
        self.mockup_stuck_joint_robustness = [None]*6
      
        self.mockup_max_latency = 0.0 # conf file
        self.sim_max_latency = 0.0 # conf file 
        self.mockup_max_velocity = 0.0 # init value
        self.mockup_max_acceleration = 0.0 # init value
        self.stuck_joint_q_difference_threshold = 0.0 # conf file
        self.max_q_error = 0.0 # conf file
        self.min_q_error = 0.0 # conf file
        self.old_mockup_data = None
      

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

        # Monitor related
        self.sample_delay = monitoring_config["sample_delay"]
        self.mockup_max_latency = monitoring_config["mockup_max_latency"]
        self.sim_max_latency = monitoring_config["sim_max_latency"]
        self.stuck_joint_q_difference_threshold = monitoring_config["stuck_joint_q_difference_threshold"]
        self.stuck_joint_qd_threshold = monitoring_config["stuck_joint_qd_threshold"]
        self.max_q_error = monitoring_config["max_q_error"]
        self.min_q_error = monitoring_config["min_q_error"]

    def initialize_monitor(self):

        # Mockup latency monitor
        mockup_latency_vars = mstlo.Variables()
        mockup_latency_vars.set("mockup_max_latency", self.mockup_max_latency)
        mockup_latency_spec = mstlo.parse_formula(self.mockup_latency_formula)
        self.mockup_latency_monitor = mstlo.Monitor(
            formula=mockup_latency_spec, semantics="DelayedQuantitative", variables=mockup_latency_vars
        )
        self._l.info(f"Monitoring service initialized with monitor: {self.mockup_latency_monitor}")

        # Simulation latency monitor
        sim_latency_vars = mstlo.Variables()
        sim_latency_vars.set("sim_max_latency", self.sim_max_latency)
        sim_latency_spec = mstlo.parse_formula(self.sim_latency_formula)
        self.sim_latency_monitor = mstlo.Monitor(
            formula=sim_latency_spec, semantics="DelayedQuantitative", variables=sim_latency_vars
        )
        self._l.info(f"Monitoring service initialized with monitor: {self.sim_latency_monitor}")

        # Mockup velocity monitor
        mockup_velocity_vars = mstlo.Variables()
        mockup_velocity_vars.set("mockup_max_velocity", self.mockup_max_velocity)
        mockup_velocity_spec = mstlo.parse_formula(self.mockup_velocity_formula)
        self.mockup_velocity_monitor = mstlo.Monitor(
            formula=mockup_velocity_spec, semantics="DelayedQuantitative", variables=mockup_velocity_vars
        )
        self._l.info(f"Monitoring service initialized with monitor: {self.mockup_velocity_monitor}")

        # Mockup acceleration monitor
        mockup_acceleration_vars = mstlo.Variables()
        mockup_acceleration_vars.set("mockup_max_acceleration", self.mockup_max_acceleration)
        mockup_acceleration_spec = mstlo.parse_formula(self.mockup_acceleration_formula)
        self.mockup_acceleration_monitor = mstlo.Monitor(
            formula=mockup_acceleration_spec, semantics="DelayedQuantitative", variables=mockup_acceleration_vars
        )
        self._l.info(f"Monitoring service initialized with monitor: {self.mockup_acceleration_monitor}")

        # Stuck joint monitor
        mockup_stuck_joint_vars = mstlo.Variables()
        mockup_stuck_joint_vars.set("stuck_joint_q_difference_threshold", self.stuck_joint_q_difference_threshold)
        mockup_stuck_joint_vars.set("stuck_joint_qd_threshold", self.stuck_joint_qd_threshold)
        self.mockup_stuck_joint_spec = mstlo.parse_formula(self.mockup_stuck_joint_formula)
        self.mockup_stuck_joint_monitor_0 = mstlo.Monitor(
            formula=self.mockup_stuck_joint_spec, semantics="DelayedQuantitative", variables=mockup_stuck_joint_vars
        )
        self.mockup_stuck_joint_monitor_1 = mstlo.Monitor(
            formula=self.mockup_stuck_joint_spec, semantics="DelayedQuantitative", variables=mockup_stuck_joint_vars
        )
        self.mockup_stuck_joint_monitor_2 = mstlo.Monitor(
            formula=self.mockup_stuck_joint_spec, semantics="DelayedQuantitative", variables=mockup_stuck_joint_vars
        )
        self.mockup_stuck_joint_monitor_3 = mstlo.Monitor(
            formula=self.mockup_stuck_joint_spec, semantics="DelayedQuantitative", variables=mockup_stuck_joint_vars
        )
        self.mockup_stuck_joint_monitor_4 = mstlo.Monitor(
            formula=self.mockup_stuck_joint_spec, semantics="DelayedQuantitative", variables=mockup_stuck_joint_vars
        )
        self.mockup_stuck_joint_monitor_5 = mstlo.Monitor(
            formula=self.mockup_stuck_joint_spec, semantics="DelayedQuantitative", variables=mockup_stuck_joint_vars
        )

        # Sim vs mockup monitor
        sim_vs_mockup_vars = mstlo.Variables()
        sim_vs_mockup_vars.set("max_q_error", self.max_q_error)
        sim_vs_mockup_vars.set("min_q_error", self.min_q_error)
        sim_vs_mockup_spec = mstlo.parse_formula(self.sim_vs_mockup_formula)
        self.sim_vs_mockup_monitor = mstlo.Monitor(
            formula=sim_vs_mockup_spec, semantics="Rosi", variables=sim_vs_mockup_vars
        )
        self._l.info(f"Monitoring service initialized with monitor: {self.mockup_tcp_monitor}")

        # Mockup tcp monitor
        mockup_tcp_vars = mstlo.Variables()
        mockup_tcp_vars.set("max_q_error", self.max_q_error)
        mockup_tcp_vars.set("min_q_error", self.min_q_error)
        mockup_tcp_spec = mstlo.parse_formula(self.mockup_tcp_formula)
        self.mockup_tcp_monitor = mstlo.Monitor(
            formula=mockup_tcp_spec, semantics="Rosi", variables=mockup_tcp_vars
        )
        self._l.info(f"Monitoring service initialized with monitor: {self.mockup_tcp_monitor}")

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
        self.initialize_monitor()
        time.sleep(10) # Let the mockup initialize

        while True:
            time.sleep(self.milliseconds_to_seconds(self.sample_delay)) # time.sleep takes seconds as parameter

            # Fetch mockup and sim data in parallel
            results = {}
            def fetch_mockup(): results["mockup"] = self.get_historical_mockup_data()
            def fetch_sim(): results["sim"] = self.get_historical_simulation_data()
            t1 = threading.Thread(target=fetch_mockup)
            t2 = threading.Thread(target=fetch_sim)
            t1.start(); t2.start()
            t1.join(); t2.join()
            mockup_data, sim_data = results["mockup"][0], results["sim"][0] # results are type list with only one element

            print("mockup_data:", mockup_data)
            print("sim_data", sim_data)

            # Extract last record for latency checks
            time_stamp = time.time()
            mockup_latency_robustness = self.compute_latency_robustness(mockup_data, "mockup_state", time_stamp)
            sim_latency_robustness = self.compute_latency_robustness(sim_data, "simulation_state", time_stamp)

            # Mockup velocity
            mockup_velocity_robustness = self.compute_mockup_velocity_robustness(mockup_data)

            # Mockup acceleration
            mockup_acceleration_robustness = self.compute_mockup_acceleration_robustness(mockup_data)


            # Mockup stuck joint detection
            for i in range(6):
                self.mockup_stuck_joint_robustness[i] = self.compute_mockup_stuck_joint(mockup_data, i)
            
            # Sim vs mockup robustness
            sim_vs_mockup_robustness = self.compute_sim_vs_mockup_robustness(mockup_data, sim_data)

            # Mockup tcp robustness
            mockup_tcp_robustness = self.compute_mockup_tcp_robustness(mockup_data)
            
            print("mockup latency robustness:", mockup_latency_robustness)
            print("sim latency robustness", sim_latency_robustness)
            print("mockup velocity robustness", mockup_velocity_robustness)
            print("mockup acceleration robustness", mockup_acceleration_robustness)

            for i in range(6):
                print(f"mockup stuck joint_{i}", self.mockup_stuck_joint_robustness[i])

            print("sim vs mockup robustness", sim_vs_mockup_robustness)
            print("mockup tcp robustness", mockup_tcp_robustness)

            # Set this after doing the computaions
            # Used in the acceleration calculation where we need the diffence in velocity and time
            self.old_mockup_data = mockup_data

            self.write_monitoring_results(
                mockup_latency_robustness,
                sim_latency_robustness,
                mockup_velocity_robustness,
                mockup_acceleration_robustness,
                self.mockup_stuck_joint_robustness,
                sim_vs_mockup_robustness,
                mockup_tcp_robustness,
            )

    
    def compute_latency_robustness(self, sample_data, measurement, time_stamp):
        if self.sim_latency_monitor is None or self.mockup_latency_monitor is None:
            raise RuntimeError("Call start_monitoring() before compute_robutsness")

        # Calculate the difference in time stamps for the mockup and the monitor service
        adj_time = time_stamp - (self.milliseconds_to_seconds(self.sample_delay))
        sample_time = sample_data["time_stamp"]
        time_diff = np.absolute(sample_time - adj_time)

        output = None

        if measurement == "mockup_state":   
            output = self.mockup_latency_monitor.update(
                signal="time_diff", value=time_diff, timestamp=adj_time
            )
        elif measurement == "simulation_state":
            output = self.sim_latency_monitor.update(
                signal="time_diff", value=time_diff, timestamp=adj_time
            )
        
        if output is None:
            return None

        return output.verdicts()
    
    def compute_mockup_velocity_robustness(self, mockup_data):
        if mockup_data is None:
            return None
        
        # Get the max velocity and the time from the mockup_data
        max_velocity = mockup_data[RobotArmStateKeys.JOINT_MAX_SPEED]
        time = mockup_data["time_stamp"]

        # Get the current velocity of each joint
        qd_list = []
        for i in range(6):
            qd_list.append(mockup_data[f"qd_actual_{i}"])
        
        # Find the joint with the largest velocity
        largest_qd = max(qd_list)

        # Update the max velocity variable
        self.mockup_velocity_monitor.get_variables().set("mockup_max_velocity", max_velocity)

        # Update the qd signal in the monitor
        output = self.mockup_velocity_monitor.update('qd', largest_qd, time)

        return output.verdicts()


    def compute_mockup_stuck_joint(self, mockup_data, joint):
        # Just checking the first one
        if self.mockup_stuck_joint_monitor_0 is None:
            raise RuntimeError("Monitor is not initialized. Call start_monitoring() first.")

        if mockup_data is None:
            return None
        
        robot_mode_from_data = mockup_data["robot_mode"]
        if robot_mode_from_data == "Running":
            self.robot_mode = 1
        else:
            self.robot_mode = 0
        
        time_stamp = mockup_data["time_stamp"]
        qd = mockup_data[f"qd_actual_{joint}"]
        q_diff = mockup_data[f"q_target_{joint}"] - mockup_data[f"q_actual_{joint}"]
       

        batch = {
            "qd" : [(qd, time_stamp)],
            "q_diff" : [(q_diff, time_stamp)],
            "robot_mode" : [(self.robot_mode, time_stamp)]
        }

        monitors = [
            self.mockup_stuck_joint_monitor_0,
            self.mockup_stuck_joint_monitor_1,
            self.mockup_stuck_joint_monitor_2,
            self.mockup_stuck_joint_monitor_3,
            self.mockup_stuck_joint_monitor_4,
            self.mockup_stuck_joint_monitor_5,
        ]

        output = monitors[joint].update_batch(batch)

        return output.verdicts()
        
    def compute_mockup_acceleration_robustness(self, mockup_data):
        if self.mockup_acceleration_monitor is None:
            raise RuntimeError("Monitor is not initialized. Call start_monitoring() first.")
        
        # Here, the last_mockup_data is also checked since in the first loop it will be None
        if mockup_data is None or self.old_mockup_data is None:
            return None
        
        # Get the max velocity and the time from the newest mockup_data
        max_acceleration = mockup_data[RobotArmStateKeys.JOINT_MAX_ACCELERATION]
        time_new = mockup_data["time_stamp"]

        # Get the time from the old mockup_data
        time_old = self.old_mockup_data["time_stamp"] 

        # Acceleration is change in velocity over change in time: a = v_1 - v_0 / t_1 - t_0
        # Create a list of the difference in velocities
        qd_diff_list = []
        for i in range(6):
            qd_diff_list.append(np.abs(mockup_data[f"qd_actual_{i}"] - self.old_mockup_data[f"qd_actual_{i}"]))

        # Find the joint with the largest velocity difference (will have largest acceleration)
        max_qd_diff = max(qd_diff_list)

        # Set a constant value if the joint is not moving (max qd diff very small), otherwise we get 'nan' from monitor
        if max_qd_diff < 0.00001:
            max_qd_diff = 0.00000001

        # Compute the differnce in time
        time_diff = time_new - time_old

        # Prevent divission by 0
        if time_diff == 0.0:
            return None

        # Compute the acceleration for that joint
        largest_qdd = max_qd_diff / time_diff

        self.mockup_acceleration_monitor.get_variables().set("mockup_max_acceleration", max_acceleration)

        output = self.mockup_acceleration_monitor.update('qdd', largest_qdd, time_new)

        return output.verdicts()
    
    def compute_sim_vs_mockup_robustness(self, mockup_data, sim_data):
        if self.sim_vs_mockup_monitor is None:
            raise RuntimeError("Monitor is not initialized. Call start_monitoring() first.")

        if mockup_data is None or sim_data is None:
            self._l.debug(f"Skipping robustness computation sim data or mockup data was None")
            return None

        # Calculate difference between mockup and sim values for q_actual
        error = sum((mockup_data[f"q_actual_{i}"] - sim_data[f"q_actual_{i}"])**2 for i in range(6))**0.5
        self._l.debug(f"Calculated eucledian distance between mockup and sim q_actual values: {error}")
        
        # Get latest time stamp from the sample data
        latest_timestamp = max(mockup_data["time_stamp"], sim_data["time_stamp"])
        output = self.sim_vs_mockup_monitor.update(
            signal="q_diff", value=error, timestamp=latest_timestamp
        )
        return output.verdicts()
    
    def compute_mockup_tcp_robustness(self, mockup_data):
        if self.mockup_tcp_monitor is None:
            raise RuntimeError("Monitor is not initialized. Call start_monitoring() first.")

        if mockup_data is None:
            self._l.debug(f"Skipping robustness computation mockup data was None")
            return None

        # Calculate difference between mockup and sim values for q_actual
        error = sum((mockup_data[f"q_actual_{i}"] - mockup_data[f"q_actual_{i}"])**2 for i in range(6))**0.5
        self._l.debug(f"Calculated eucledian distance between mockup and sim q_actual values: {error}")
        
        # Get the time stamp from mockup data
        time_stamp = mockup_data["time_stamp"]
        output = self.mockup_tcp_monitor.update(
            signal="q_diff", value=error, timestamp=time_stamp
        )

        return output.verdicts()
        
    def get_historical_mockup_data(self):
        query = f'''
        from(bucket: "{self.influxdb_bucket}")
        |> range(start: -5000ms, stop: -{self.sample_delay}ms)
        |> filter(fn: (r) => r._measurement == "mockup_state")
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
        |> last(column: "_time")
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
    
    def get_historical_simulation_data(self):
        query = f'''
        from(bucket: "{self.influxdb_bucket}")
        |> range(start: -5000ms, stop: -{self.sample_delay}ms)
        |> filter(fn: (r) => r._measurement == "simulation_state")
        |> filter(fn: (r) => contains(value: r._field, set: [
            "q_actual_0","q_actual_1","q_actual_2","q_actual_3","q_actual_4","q_actual_5",
            "qd_actual_0","qd_actual_1","qd_actual_2","qd_actual_3","qd_actual_4","qd_actual_5",
            "tcp_pose_0","tcp_pose_1","tcp_pose_2","tcp_pose_3","tcp_pose_4","tcp_pose_5",
            "robot_mode"
        ]))
        |> group(columns: ["_measurement", "_field"])
        |> pivot(rowKey: ["_time", "_measurement"], columnKey: ["_field"], valueColumn: "_value")
        |> sort(columns: ["_time"])
        |> last(column: "_time")
        '''
        tables = self.query_api.query(query, org=self.influx_db_org)

        data = []
        fields = [f"q_actual_{i}" for i in range(6)]
        fields1 = [f"tcp_pose_{i}" for i in range(6)]
        fields2 = [f"qd_actual_{i}" for i in range(6)]
        fields3 = ["robot_mode"]

        for table in tables:
            for record in table.records:
                entry = {
                    "time_stamp": record.get_time().timestamp(),
                    **{field: record.values.get(field) for field in fields},
                    **{field: record.values.get(field) for field in fields1},
                    **{field: record.values.get(field) for field in fields2},
                    **{field: record.values.get(field) for field in fields3}
                }
                data.append(entry)

        self._l.debug(f"Retrieved a historical sample from InfluxDB: {data}.")

        return data
    
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

    def create_monitoring_msg(self,
                              mockup_latency_robustness,
                              sim_latency_robustness,
                              mockup_velocity_robustness,
                              mockup_acceleration_robustness,
                              mockup_stuck_joint_robustness,
                              sim_vs_mockup_robustness,
                              mockup_tcp_robustness):
        timestamp = datetime.fromtimestamp(time.time(), timezone.utc).isoformat()
        fields = {}

        def latest(verdicts):
            """Return the value of the last verdict, or None if no verdicts."""
            if not verdicts:
                return None
            return verdicts[-1][1]

        # Scalar robustness values (DelayedQuantitative → single float)
        if (v := latest(mockup_latency_robustness)) is not None:
            fields["mockup_latency_robustness"] = v
        if (v := latest(sim_latency_robustness)) is not None:
            fields["sim_latency_robustness"] = v
        if (v := latest(mockup_velocity_robustness)) is not None:
            fields["mockup_velocity_robustness"] = v
        if (v := latest(mockup_acceleration_robustness)) is not None:
            fields["mockup_acceleration_robustness"] = v

        # Per-joint stuck joint robustness (one field per joint)
        for i, stuck in enumerate(mockup_stuck_joint_robustness):
            if (v := latest(stuck)) is not None:
                fields[f"stuck_joint_robustness_{i}"] = v

        # Rosi robustness values (tuple → lower and upper bound fields)
        if (v := latest(sim_vs_mockup_robustness)) is not None:
            fields["sim_vs_mockup_robustness_lower"] = v[0]
            fields["sim_vs_mockup_robustness_upper"] = v[1]
        if (v := latest(mockup_tcp_robustness)) is not None:
            fields["mockup_tcp_robustness_lower"] = v[0]
            fields["mockup_tcp_robustness_upper"] = v[1]

        return {
            "measurement": "monitoring_robustness",
            "time": timestamp,
            "tags": {"source": "monitor_service"},
            "fields": fields,
        }

    def write_monitoring_results(self,
                                 mockup_latency_robustness,
                                 sim_latency_robustness,
                                 mockup_velocity_robustness,
                                 mockup_acceleration_robustness,
                                 mockup_stuck_joint_robustness,
                                 sim_vs_mockup_robustness,
                                 mockup_tcp_robustness):
        msg = self.create_monitoring_msg(
            mockup_latency_robustness,
            sim_latency_robustness,
            mockup_velocity_robustness,
            mockup_acceleration_robustness,
            mockup_stuck_joint_robustness,
            sim_vs_mockup_robustness,
            mockup_tcp_robustness,
        )
        self.record_message(msg)
