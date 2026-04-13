from influxdb_client.client.influxdb_client import InfluxDBClient
from startup.utils.logging_config import create_service_logger

class DBQueryService:
    def __init__(self):
        self.client = None
        self.query_api = None
        self.org = None
        self.bucket = None
        self.aggregate_window_size = 50  # ms
        self._l = create_service_logger("db_query_service")
        
    def setup(self, influxdb_config):
        self._l.info(f"DBQueryService setup with config: org={influxdb_config.get('org')}, bucket={influxdb_config.get('bucket')}")
        
        try:
            self.client = InfluxDBClient(**influxdb_config)
            self.query_api = self.client.query_api()
            self.org = influxdb_config["org"]
            self.bucket = influxdb_config["bucket"]
            
            if "aggregate_window_size" in influxdb_config:
                self.aggregate_window_size = influxdb_config["aggregate_window_size"]
            
            self._l.info("DBQueryService initialized successfully")
        except Exception as e:
            self._l.error(f"Failed to initialize DBQueryService: {e}", exc_info=True)
            raise

    def get_mockup_values(self, start=None, stop=None, window=None):
        if start is None:
            start = "-20s"
        if stop is None:
            stop = "now()"
        if window is None:
            window = f"{int(self.aggregate_window_size)}ms"
        
        self._l.debug(f"Querying mockup values: start={start}, stop={stop}, window={window}")
        
        query = f'''
        from(bucket: "{self.bucket}")
        |> range(start: {start}, stop: {stop})
        |> filter(fn: (r) => r._measurement == "mockup_state")
        |> filter(fn: (r) => contains(value: r._field, set: [
            "q_actual_0", "q_actual_1", "q_actual_2", "q_actual_3", "q_actual_4", "q_actual_5",
            "tcp_pose_0", "tcp_pose_1", "tcp_pose_2", "tcp_pose_3", "tcp_pose_4", "tcp_pose_5",
        ]))
        |> aggregateWindow(every: {window}, fn: last, createEmpty: true)
        |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
        |> sort(columns: ["_time"])
        '''
        
        return self._parse_query_results(query)

    def get_simulation_values(self, start=None, stop=None, window=None):
        if start is None:
            start = "-20s"
        if stop is None:
            stop = "now()"
        if window is None:
            window = f"{int(self.aggregate_window_size)}ms"
        
        self._l.debug(f"Querying simulation values: start={start}, stop={stop}, window={window}")
        
        query = f'''
        from(bucket: "{self.bucket}")
        |> range(start: {start}, stop: {stop})
        |> filter(fn: (r) => r._measurement == "simulation_state")
        |> filter(fn: (r) => contains(value: r._field, set: [
            "q_actual_0", "q_actual_1", "q_actual_2", "q_actual_3", "q_actual_4", "q_actual_5",
            "tcp_pose_0", "tcp_pose_1", "tcp_pose_2", "tcp_pose_3", "tcp_pose_4", "tcp_pose_5",
        ]))
        |> aggregateWindow(every: {window}, fn: last, createEmpty: true)
        |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
        |> sort(columns: ["_time"])
        '''
        
        return self._parse_query_results(query)

    def get_all_values(self, start=None, stop=None, window=None):
        if start is None:
            start = "-20s"
        if stop is None:
            stop = "now()"
        if window is None:
            window = f"{int(self.aggregate_window_size)}ms"
        
        self._l.debug(f"Querying all values: start={start}, stop={stop}, window={window}")
        
        query = f'''
        from(bucket: "{self.bucket}")
        |> range(start: {start}, stop: {stop})
        |> filter(fn: (r) => 
            r._measurement == "mockup_state" or 
            r._measurement == "simulation_state"
        )
        |> filter(fn: (r) => contains(value: r._field, set: [
            "q_actual_0", "q_actual_1", "q_actual_2", "q_actual_3", "q_actual_4", "q_actual_5",
            "tcp_pose_0", "tcp_pose_1", "tcp_pose_2", "tcp_pose_3", "tcp_pose_4", "tcp_pose_5",
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
        fields_tcp = [f"tcp_pose_{i}" for i in range(6)]
        measurements = ["mockup_state", "simulation_state"]
        
        for table in tables:
            for record in table.records:
                entry = {
                    "time_stamp": record.get_time().isoformat(),
                }
                for measurement in measurements:
                    for field in fields + fields_tcp:
                        column_key = f"{measurement}_{field}"
                        if column_key in record.values and record.values[column_key] is not None:
                            entry[f"{measurement}_{field}"] = record.values[column_key]
                
                data.append(entry)
        
        self._l.debug(f"Retrieved {len(data)} samples from InfluxDB")
        return data

    def get_measurements(self, start=None):
        if start is None:
            start = "-1h"
        
        query = f'''
        import "influxdata/influxdb/schema"
        schema.measurements(bucket: "{self.bucket}")
        '''
        
        tables = self.query_api.query(query, org=self.org)
        measurements = []
        
        for table in tables:
            for record in table.records:
                measurements.append(record.get_value())
        
        self._l.debug(f"Available measurements: {measurements}")
        return measurements

    def _parse_query_results(self, query):
        try:
            tables = self.query_api.query(query, org=self.org)
            data = []
            fields = [f"q_actual_{i}" for i in range(6)]
            fields_tcp = [f"tcp_pose_{i}" for i in range(6)]
            
            for table in tables:
                for record in table.records:
                    entry = {
                        "time_stamp": record.get_time().isoformat(),
                    }
                    for field in fields + fields_tcp:
                        if field in record.values and record.values[field] is not None:
                            entry[field] = record.values[field]
                    
                    data.append(entry)
            
            self._l.debug(f"Retrieved {len(data)} samples from InfluxDB")
            return data
        except Exception as e:
            self._l.error(f"Failed to parse query results: {e}", exc_info=True)
            return []

    def close(self):
        if self.client is not None:
            try:
                self.client.close()
                self._l.info("DBQueryService client closed")
            except Exception as e:
                self._l.warning(f"Error closing client: {e}")
