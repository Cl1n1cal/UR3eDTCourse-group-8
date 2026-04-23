from influxdb_client.client.influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from communication.protocol import ROUTING_KEY_STATE, ROUTING_KEY_MONITORING
from communication.factory import RabbitMQFactory
from startup.utils.logging_config import create_service_logger
import threading
import mstlo_python as mstlo

class MonitoringService:
    def __init__(self):
        self.write_api = None
        self.influx_db_org = None
        self.influxdb_bucket = None
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self._l = create_service_logger("monitoring_service")
        self._monitor = None
        self._max_error = 0.1
        self._min_error = -0.1
        self._spec_formula = "(q_diff < $max_error && q_diff > $min_error)"

    def _initialize_monitor(self):
        spec_vars = mstlo.Variables()
        spec_vars.set("max_error", self._max_error)
        spec_vars.set("min_error", self._min_error)
        spec = mstlo.parse_formula(self._spec_formula)
        self._monitor = mstlo.Monitor(
            formula=spec, semantics="Rosi", variables=spec_vars
        )
        self._l.info(f"Monitoring service initialized with monitor: {self._monitor}")

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

    def setup(self, monitoring_config):
        self._l.info("Monitoring setup with config ", monitoring_config)
        self.rabbitmq.connect_to_server()

        client = InfluxDBClient(**monitoring_config) 
        self.write_api = client.write_api(write_options=SYNCHRONOUS)
        self.query_api = client.query_api()
        self.influx_db_org = monitoring_config["org"]
        self.influxdb_bucket = monitoring_config["bucket"]
        self.sample_delay = monitoring_config["sample_delay"]

        self.rabbitmq.subscribe(routing_key=ROUTING_KEY_STATE,
                        on_message_callback=self.process_state_sample)

    def start_monitoring(self):
        if self.rabbitmq == None:
            return
        try:
            def run():
                if self.rabbitmq == None:
                    return
                self._initialize_monitor()
                self.rabbitmq.start_consuming()

            self.thread = threading.Thread(target=run, daemon=False)
            self.thread.start()
        except KeyboardInterrupt:
            self.rabbitmq.close()

    def process_state_sample(self, ch, method, properties, body_json):
        # get robot arm state history from influx db
        data = self.get_historical_data()
        robustness = self.compute_robustness(data)
        if robustness is None:
            return
        self._l.debug(f"Computed robustness: {robustness}")
        self.record_message(self.create_robustness_recorder_msg(robustness, "q_diff_robustness"))
        #self.rabbitmq.send_message(routing_key=ROUTING_KEY_MONITORING, message=self.create_robustness_msg(robustness, "q_diff_robustness"))
        
    def get_historical_data(self):
        query = f'''
        from(bucket: "{self.influxdb_bucket}")
        |> range(start: -1h, stop: -{self.sample_delay}ms)
        |> filter(fn: (r) => r._measurement == "mockup_state" or r._measurement == "simulation_state")
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
    
    def compute_robustness(self, sample_data):
        if self._monitor is None:
            raise RuntimeError("Monitor is not initialized. Call start_monitoring() first.")

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
        return robustness.verdicts()
    
    def create_robustness_recorder_msg(self, robustness, measurement_name: str):
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
    