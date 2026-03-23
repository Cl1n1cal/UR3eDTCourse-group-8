from influxdb_client.client.influxdb_client import InfluxDBClient
from startup.utils.logging_config import create_service_logger
import logging
import numpy as np

class CalibrationService:
    def __init__(self):
        self.influx_db_org = None
        self.influxdb_bucket = None
        self.rabbitmq = None
        self._l = create_service_logger("calibration_service", level=logging.DEBUG)

    def setup(self, influxdb_config):
        self._l.info("InfluxDBRecorder setup with config ", influxdb_config)
        self.client = InfluxDBClient(**influxdb_config)
        self.query_api = self.client.query_api()
        self.bucket = influxdb_config["bucket"]
        self.org = influxdb_config["org"]

    def get_motion_data(self, start="-10s"):
        query = f'''
        from(bucket: "{self.bucket}")
        |> range(start: {start})
        |> filter(fn: (r) => r._measurement == "mockup_state")
        |> filter(fn: (r) => r._field == "qd_acual_3")
        |> filter(fn: (r) => r.source == "mockup_state_publisher")
        |> sort(columns: ["_time"])
        '''
        
        tables = self.query_api.query(query, org=self.org)

        times = []
        positions = []

        for table in tables:
            for record in table.records:
                times.append(record.get_time())
                positions.append(record.get_value())

        return np.array(times), np.array(positions)
    
    def estimate_duration(self, times, positions, threshold=1e-3):
        vel = np.gradient(positions)

        moving = np.abs(vel) > threshold

        indices = np.where(moving)[0]
        if len(indices) == 0:
            return None

        t_start = times[indices[0]]
        t_end = times[indices[-1]]

        return (t_end - t_start).total_seconds()
    
    def estimate_T(self):
        times, positions = self.get_motion_data()
        print(f"Times: {times}, positions: {positions}")
        T = self.estimate_duration(times, positions)
        return T

    def get_motion_duration(times, positions, threshold=1e-3):
        """
        Compute the start, end, and duration of motion.

        times: np.array of datetime objects
        positions: np.array of floats
        threshold: minimum position considered as motion
        """
        # Find indices where motion occurs
        motion_indices = np.where(np.abs(positions) > threshold)[0]
        
        if len(motion_indices) == 0:
            return None, None, 0  # no motion detected
        
        start_time = times[motion_indices[0]]
        end_time = times[motion_indices[-1]]
        duration = (end_time - start_time).total_seconds()
        
        return start_time, end_time, duration