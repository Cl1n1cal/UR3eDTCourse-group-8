from communication.factory import RabbitMQFactory
from communication.protocol import ROUTING_KEY_CTRL, CtrlMsgKeys, CtrlMsgFields
import numpy as np
import time
from influxdb_client.client.influxdb_client import InfluxDBClient
from startup.utils.logging_config import create_service_logger
from communication.factory import RabbitMQFactory, ROUTING_KEY_CALIBRATION
import logging
import time
import numpy as np
from models.robot_model import create_robot
from scipy.optimize import least_squares
from utils.calculation_functions import se3_to_pos_rpy
from utils.configuration import load_config
import json
from datetime import datetime


class ParticleFilter:
    def __init__(self):
        self.influx_db_org = None
        self.influxdb_bucket = None
        #self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self.time_interval = 0
        self.robot = None
        self.max_pos_error = 0.01 # 1cm
        self.max_rot_error = 0.017 # ~1 degree in radians
        self.aggregate_window_size = 50 # ms
        self._l = create_service_logger("ParticleFilter")

    def setup(self, calibration_config):
        self._l.info("ParticleFilter service setup with config ", calibration_config)
        self.client = InfluxDBClient(**calibration_config)
        #self.rabbitmq.connect_to_server()
        self.query_api = self.client.query_api()
        self.bucket = calibration_config["bucket"]
        self.org = calibration_config["org"]
        self.time_interval = calibration_config["time_interval"]
        self.max_pos_error = calibration_config["max_position_error"]
        self.max_rot_error = calibration_config["max_rotation_error"]
        self.aggregate_window_size = calibration_config["aggregate_window_size"]
        self.dh_guess = np.array(calibration_config["initial_guess"]["d"] + calibration_config["initial_guess"]["a"] + calibration_config["initial_guess"]["alpha"])

    def get_mockup_values(self, start=None, stop=None):
        if start is None:
            start = f"-{self.time_interval}s"
        if stop is None:
            stop = "now()"
        window = f"{int(self.aggregate_window_size)}ms"
        query = f'''
        from(bucket: "{self.bucket}")
        |> range(start: {start}, stop: {stop})
        |> filter(fn: (r) => r._measurement == "mockup_state")
        |> filter(fn: (r) => contains(value: r._field, set: [
            "q_actual_0",
            "q_actual_1",
            "q_actual_2",
            "q_actual_3",
            "q_actual_4",
            "q_actual_5",
            "qd_actual_0",
            "qd_actual_1",
            "qd_actual_2",
            "qd_actual_3",
            "qd_actual_4",
            "qd_actual_5",
        ]))
        |> aggregateWindow(every: {window}, fn: last, createEmpty: true)
        |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
        |> sort(columns: ["_time"])
        '''
        
        tables = self.query_api.query(query, org=self.org)

        data = []
        fields = [f"q_actual_{i}" for i in range(6)]
        fields1 = [f"qd_actual_{i}" for i in range(6)]

        for table in tables:
            for record in table.records:
                entry = {
                    "time_stamp": record.get_time().isoformat(),
                    **{field: record.values.get(field) for field in fields},
                    **{field: record.values.get(field) for field in fields1}
                }
                data.append(entry)

        self._l.debug(f"Retrieved {len(data)} samples from InfluxDB for calibration.")
        
        return data
    
    def get_simulation_values(self, start=None, stop=None):
        if start is None:
            start = f"-{self.time_interval}s"
        if stop is None:
            stop = "now()"
        window = f"{int(self.aggregate_window_size)}ms"
        query = f'''
        from(bucket: "{self.bucket}")
        |> range(start: {start}, stop: {stop})
        |> filter(fn: (r) => r._measurement == "simulation_state")
        |> filter(fn: (r) => contains(value: r._field, set: [
            "q_actual_0",
            "q_actual_1",
            "q_actual_2",
            "q_actual_3",
            "q_actual_4",
            "q_actual_5",
            "qd_actual_0",
            "qd_actual_1",
            "qd_actual_2",
            "qd_actual_3",
            "qd_actual_4",
            "qd_actual_5",
        ]))
        |> aggregateWindow(every: {window}, fn: last, createEmpty: true)
        |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
        |> sort(columns: ["_time"])
        '''
        
        tables = self.query_api.query(query, org=self.org)

        data = []
        fields = [f"q_actual_{i}" for i in range(6)]
        fields1 = [f"qd_actual_{i}" for i in range(6)]

        for table in tables:
            for record in table.records:
                entry = {
                    "time_stamp": record.get_time().isoformat(),
                    **{field: record.values.get(field) for field in fields},
                    **{field: record.values.get(field) for field in fields1}
                }
                data.append(entry)

        self._l.debug(f"Retrieved {len(data)} samples from InfluxDB for calibration.")
        
        return data
    
    def debug_measurements(self):
        """Check what measurements exist in the database"""
        start = f"-{self.time_interval}s"
        query = f'''
            from(bucket: "{self.bucket}")
            |> range(start: {start})
            |> group(columns: ["_measurement"])
            |> distinct(column: "_measurement")
            '''
        tables = self.query_api.query(query, org=self.org)
        measurements = []
        for table in tables:
            for record in table.records:
                measurements.append(record.get_value())
        print(f"Available measurements: {measurements}")
        return measurements

    def get_all_values(self):
        start = f"-{self.time_interval}s"
        window = f"{int(self.aggregate_window_size)}ms"
        query = f'''
            from(bucket: "{self.bucket}")
            |> range(start: {start})
            |> filter(fn: (r) => 
                r._measurement == "mockup_state" or 
                r._measurement == "simulation_state"
            )
            |> filter(fn: (r) => contains(value: r._field, set: [
                "q_actual_0",
                "q_actual_1",
                "q_actual_2",
                "q_actual_3",
                "q_actual_4",
                "q_actual_5",
                "qd_actual_0",
                "qd_actual_1",
                "qd_actual_2",
                "qd_actual_3",
                "qd_actual_4",
                "qd_actual_5",
                "q_target_0",
                "q_target_1",
                "q_target_2",
                "q_target_3",
                "q_target_4",
                "q_target_5",
            ]))
            |> aggregateWindow(every: {window}, fn: last, createEmpty: false)
            |> pivot(
                rowKey: ["_time"], 
                columnKey: ["_measurement", "_field"], 
                valueColumn: "_value"
            )
            |> sort(columns: ["_time"])
            '''
        
        tables = self.query_api.query(query, org=self.org)

        data = []
        fields = [f"q_actual_{i}" for i in range(6)]
        fields1 = [f"tcp_pose_{i}" for i in range(6)]
        measurements = ["mockup_state", "simulation_state"]

        for table in tables:
            for record in table.records:
                entry = {
                    "time_stamp": record.get_time().isoformat(),
                }
                # Extract fields for each measurement (prioritize mockup_state)
                for measurement in measurements:
                    for field in fields + fields1:
                        column_key = f"{measurement}_{field}"
                        if column_key in record.values and record.values[column_key] is not None:
                            entry[f"{measurement}_{field}"] = record.values[column_key]
                
                data.append(entry)

        self._l.debug(f"Retrieved {len(data)} samples from InfluxDB for calibration.")
        
        return data


p_filter = ParticleFilter()
config = load_config("startup/startup.conf")
p_filter.setup(calibration_config=config["calibration_service"])

start = "2026-04-15T17:20:00+02:00"
stop = "2026-04-15T17:20:40+02:00"

# Parse ISO 8601 format directly
dt_start = datetime.fromisoformat(start)
dt_stop = datetime.fromisoformat(stop)

# Already in ISO 8601 format, but can be used as-is
start = dt_start.isoformat()
stop = dt_stop.isoformat()

sim_data = p_filter.get_simulation_values(start=start, stop=stop)
mockup_data = p_filter.get_mockup_values(start=start, stop=stop)

# Properly close the InfluxDB client to prevent cleanup errors
p_filter.client.close()

with open("mockup_results.json", "w") as file:
    json.dump(mockup_data, file, indent=4)

with open("sim_results.json", "w") as file:
    json.dump(sim_data, file, indent=4)

def round_data(filename, output_filename):
    with open(filename, "r") as file:
        data = json.load(file)
    
    for item in data:
        for key in item:
            if key != "time_stamp" and isinstance(item[key], (int, float)):
                item[key] = round(item[key], 4)
    
    with open(output_filename, "w") as file:
        json.dump(data, file, indent=4)
    
    print(f"Processed {filename} → {output_filename}")

# Process both files
round_data("mockup_results.json", "mockup_results_rounded.json")
round_data("sim_results.json", "sim_results_rounded.json")