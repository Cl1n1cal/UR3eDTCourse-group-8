import json
import math
import os
import time
import threading
from datetime import datetime, timezone

_LIFECYCLE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'joint_lifecycle.json') # fallback when InfluxDB wiped

from influxdb_client.client.influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS

from communication.factory import RabbitMQFactory
from communication.protocol import RobotArmStateKeys, ROUTING_KEY_STATE, ROUTING_KEY_JOINT_ROTATIONS, ROUTING_KEY_RECORDER
from startup.utils.logging_config import create_service_logger

NUM_JOINTS = 6

class JointRotationCounterService:
    def __init__(self):
        self.query_api = None
        self.influx_db_org = None
        self.influx_db_bucket = None

        self.rabbitmq_publisher = RabbitMQFactory.create_rabbitmq()
        self.rabbitmq_consumer = RabbitMQFactory.create_rabbitmq()

        self._rotations: list[float] = [0.0] * NUM_JOINTS
        self._prev_q: list[float] | None = None

        self.rotation_threshold: float = 0.0   # 0 means disabled
        self.publish_interval: float = 5.0
        self._threshold_warned: list[bool] = [False] * NUM_JOINTS

        self._lock = threading.Lock()
        self._l = create_service_logger("joint_rotation_counter_service")

    def setup(self, config: dict) -> None:
        self._l.info("JointRotationCounterService setup with config", config)

        client = InfluxDBClient(**config)
        self.query_api = client.query_api()
        self.influx_db_org = config["org"]
        self.influx_db_bucket = config["bucket"]
        
        self.rotation_threshold = float(config.get("rotation_threshold", 0.0))
        self.publish_interval = float(config.get("publish_interval", 5.0))

        self.rabbitmq_publisher.connect_to_server()
        self.rabbitmq_consumer.connect_to_server()
        self.rabbitmq_consumer.subscribe(routing_key=ROUTING_KEY_STATE, on_message_callback=self._on_state_message)

        self._load_persisted_rotations()

    def _load_persisted_rotations(self) -> None:
        """Load rotation counts from InfluxDB, falling back to JSON file."""
        loaded_from = None

        try:
            query = f'''
            from(bucket: "{self.influx_db_bucket}")
              |> range(start: -90d)
              |> filter(fn: (r) => r._measurement == "joint_rotations")
              |> last()
              |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
            '''
            tables = self.query_api.query(query, org=self.influx_db_org)
            found_any = False
            for table in tables:
                for record in table.records:
                    for i in range(NUM_JOINTS):
                        key = f"joint_{i}_rotations"
                        val = record.values.get(key)
                        if val is not None:
                            self._rotations[i] = max(self._rotations[i], float(val))
                            found_any = True
            if found_any:
                loaded_from = "InfluxDB"
        except Exception as e:
            self._l.warning(f"InfluxDB load failed: {e}")

        # fallback to json file
        if loaded_from is None:
            try:
                with open(_LIFECYCLE_FILE) as f:
                    data = json.load(f)
                for i in range(NUM_JOINTS):
                    saved = data.get(f"joint_{i}_rotations", 0.0)
                    self._rotations[i] = max(self._rotations[i], float(saved))
                loaded_from = "JSON file"
            except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError):
                pass

        total = sum(self._rotations)
        src = loaded_from or "zero (first run)"
        self._l.info(
            f"Loaded persisted rotations from {src}: "
            f"{[round(r, 3) for r in self._rotations]} (total = {total:.3f} rev)"
        )

    def _save_lifecycle_json(self) -> None:
        """Write current rotation totals to the JSON fallback file."""
        try:
            data = {f"joint_{i}_rotations": self._rotations[i] for i in range(NUM_JOINTS)}
            data["total_rotations"] = sum(self._rotations)
            with open(_LIFECYCLE_FILE, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self._l.warning(f"Could not write lifecycle JSON: {e}")

    def _on_state_message(self, ch, method, properties, message: dict) -> None:
        q_actual = message.get(RobotArmStateKeys.Q_ACTUAL)
        if q_actual is None or len(q_actual) != NUM_JOINTS:
            return

        with self._lock:
            if self._prev_q is not None:
                for i in range(NUM_JOINTS):
                    delta = abs(q_actual[i] - self._prev_q[i])
                    self._rotations[i] += delta / (2.0 * math.pi)

                    if (
                        self.rotation_threshold > 0
                        and self._rotations[i] >= self.rotation_threshold
                        and not self._threshold_warned[i]
                    ):
                        self._l.warning(
                            f"Joint {i} has reached {self._rotations[i]:.1f} rotations "
                            f"(threshold = {self.rotation_threshold:.1f}). "
                            "Consider inspection / maintenance."
                        )
                        self._threshold_warned[i] = True

            self._prev_q = list(q_actual)

    def _publish_loop(self) -> None:
        while True:
            time.sleep(self.publish_interval)
            try:
                with self._lock:
                    rotations_snapshot = list(self._rotations)
                    warnings = list(self._threshold_warned)

                ts = datetime.now(timezone.utc).isoformat()
                total_rotations = sum(rotations_snapshot)

                fields = {f"joint_{i}_rotations": float(rotations_snapshot[i]) for i in range(NUM_JOINTS)}
                fields["total_rotations"] = float(total_rotations)

                msg = {
                    "timestamp": ts,
                    "total_rotations": round(total_rotations, 4),
                    "joint_rotations": [round(r, 4) for r in rotations_snapshot],
                    "threshold": self.rotation_threshold,
                    "threshold_exceeded": [
                        self.rotation_threshold > 0 and r >= self.rotation_threshold
                        for r in rotations_snapshot
                    ],
                }
                self.rabbitmq_publisher.send_message(ROUTING_KEY_JOINT_ROTATIONS, msg)

                self._l.info(
                    f"Joint rotations: {[round(r, 2) for r in rotations_snapshot]} "
                    f"| Total: {total_rotations:.2f} rev"
                )

                self.rabbitmq_publisher.send_message(
                    ROUTING_KEY_RECORDER,
                    {
                        "measurement": "joint_rotations",
                        "tags": {"source": "joint_rotation_counter_service"},
                        "time": ts,
                        "fields": fields,
                    },
                )
                self._save_lifecycle_json()

            except Exception as e:
                self._l.warning(f"Publish / persist failed: {e}", exc_info=e)

    def start_serving(self) -> None:
        threading.Thread(target=self._publish_loop, daemon=True).start()
        self._l.info(
            f"JointRotationCounterService started "
            f"(threshold={self.rotation_threshold} rev, "
            f"publish_interval={self.publish_interval}s)."
        )
        try:
            self.rabbitmq_consumer.start_consuming()
        except KeyboardInterrupt:
            self.cleanup()

    def cleanup(self) -> None:
        self.rabbitmq_consumer.close()
        self.rabbitmq_publisher.close()
