from influxdb_client.client.influxdb_client import InfluxDBClient
from startup.utils.logging_config import create_service_logger
import logging
import numpy as np
import time
from datetime import datetime, timezone
import math
import threading

class CalibrationService:
    def __init__(self):
        self.influx_db_org = None
        self.influxdb_bucket = None
        self.rabbitmq = None
        self.time_interval = 0
        self._l = create_service_logger("calibration_service", level=logging.DEBUG)

    def setup(self, calibration_config):
        self._l.info("Calibration service setup with config ", calibration_config)
        self.client = InfluxDBClient(**calibration_config)
        self.query_api = self.client.query_api()
        self.bucket = calibration_config["bucket"]
        self.org = calibration_config["org"]
        self.time_interval = calibration_config["time_interval"]

    def get_mockup_values(self):
        start = f"-{self.time_interval}s"
        query = f'''
        from(bucket: "{self.bucket}")
        |> range(start: {start})
        |> filter(fn: (r) => r._measurement == "mockup_state")
        |> filter(fn: (r) => contains(value: r._field, set: [
            "q_actual_0",
            "q_actual_1",
            "q_actual_2",
            "q_actual_3",
            "q_actual_4",
            "q_actual_5",
            "tcp_pose_0",
            "tcp_pose_1",
            "tcp_pose_2",
            "tcp_pose_3",
            "tcp_pose_4",
            "tcp_pose_5",
        ]))
        |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
        |> sort(columns: ["_time"])
        '''
        
        tables = self.query_api.query(query, org=self.org)


        data = []
        fields = [f"q_actual_{i}" for i in range(6)]
        fields1 = [f"tcp_pose_{i}" for i in range(6)]

        for table in tables:
            for record in table.records:
                entry = {
                    "time_stamp": record.get_time().isoformat(),
                    **{field: record.values.get(field) for field in fields},
                    **{field: record.values.get(field) for field in fields1}
                }
                data.append(entry)

        return data
    

    def do_some_logic(self, mockup_vals):
        pass # TODO

    def start_serving(self):
        def _calibration_loop():
            while True:
                time.sleep(self.time_interval)
                mockup_vals = self.get_mockup_values()
                self.do_some_logic(mockup_vals)

        calibration_thread = threading.Thread(target=_calibration_loop, daemon=True)
        calibration_thread.start()