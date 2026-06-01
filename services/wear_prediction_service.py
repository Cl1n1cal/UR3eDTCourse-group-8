import os
import time
import threading
import pickle
import multiprocessing
import numpy as np
from collections import deque
from datetime import datetime, timezone

from influxdb_client.client.influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS

from communication.factory import RabbitMQFactory
from communication.protocol import RobotArmStateKeys, RobotMode, ROUTING_KEY_STATE, ROUTING_KEY_WEAR
from models.robot_model import create_robot
from utils.calculation_functions import se3_to_pos_rpy
from startup.utils.logging_config import create_service_logger

NUM_JOINTS = 6
DEFAULT_MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ml_models", "wear_model.pkl")


def _safe_n_jobs(default: int = -1) -> int:
    process = multiprocessing.current_process()
    if process.name != "MainProcess" or process.daemon:
        return 1
    return default


def _set_n_jobs_recursive(model, n_jobs: int) -> None:
    if hasattr(model, "n_jobs"):
        try:
            model.n_jobs = n_jobs
        except Exception:
            pass

    steps = getattr(model, "steps", None)
    if steps:
        for _, step in steps:
            _set_n_jobs_recursive(step, n_jobs)


class WearPredictionService:
    def __init__(self):
        self._l = create_service_logger("wear_prediction_service")

        self.rabbitmq_consumer = RabbitMQFactory.create_rabbitmq()
        self.rabbitmq_publisher = RabbitMQFactory.create_rabbitmq()

        self.write_api = None
        self.influx_db_org = None
        self.influx_db_bucket = None

        self.robot = None

        self.window_size: int = 40
        self._window: deque = deque()
        self._lock = threading.Lock()

        self._model = None
        self._model_path: str = DEFAULT_MODEL_PATH

        self.publish_interval: float = 2.0

        self._latest_wear: float = 0.0
        self._latest_tcp_discrepancy: float = 0.0


    def setup(self, config: dict) -> None:
        self._l.info("WearPredictionService setup with config", config)

        influx_cfg = {k: v for k, v in config.items() if k in ("url", "token", "org", "bucket")}
        client = InfluxDBClient(**influx_cfg)
        self.write_api = client.write_api(write_options=SYNCHRONOUS)
        self.influx_db_org = config["org"]
        self.influx_db_bucket = config["bucket"]

        self.window_size = int(config.get("window_size", 40))
        self.publish_interval = float(config.get("publish_interval", 2.0))
        self._model_path = config.get("model_path", DEFAULT_MODEL_PATH)

        dh = config.get("dh_parameters", {})
        d = dh.get("d", [0.1519, 0, 0, 0.11235, 0.08535, 0.0819])
        a = dh.get("a", [0, -0.24365, -0.21325, 0, 0, 0])
        alpha = dh.get("alpha", [1.57, 0.0, 0.0, 1.57, -1.57, 0.0])
        self.robot = create_robot(d, a, alpha)
        self._l.info("Robot model created for TCP discrepancy computation.")

        self._try_load_model()

        self.rabbitmq_publisher.connect_to_server()
        self.rabbitmq_consumer.connect_to_server()
        self.rabbitmq_consumer.subscribe(
            routing_key=ROUTING_KEY_STATE,
            on_message_callback=self._on_state_message,
        )


    def _try_load_model(self) -> None:
        if os.path.exists(self._model_path):
            try:
                with open(self._model_path, "rb") as f:
                    self._model = pickle.load(f)
                _set_n_jobs_recursive(self._model, _safe_n_jobs())
                self._l.info(f"Loaded wear model from {self._model_path}")
            except Exception as e:
                self._l.warning(f"Could not load wear model: {e}. Running in raw-discrepancy mode.")
        else:
            self._l.warning(f"No model found at {self._model_path}. Running in raw-discrepancy mode until model is trained.")


    def _on_state_message(self, ch, method, properties, message: dict) -> None:
        q_actual = message.get(RobotArmStateKeys.Q_ACTUAL)
        tcp_pose = message.get(RobotArmStateKeys.TCP_POSE)
        mode = message.get(RobotArmStateKeys.ROBOT_MODE)

        if q_actual is None or tcp_pose is None:
            return
        if len(q_actual) != NUM_JOINTS or len(tcp_pose) != 6:
            return

        try:
            model_tcp = se3_to_pos_rpy(self.robot.fkine(np.array(q_actual)))
        except Exception:
            return

        mockup_tcp = np.array(tcp_pose)
        discrepancy = mockup_tcp - model_tcp

        with self._lock:
            self._window.append({
                "discrepancy": discrepancy,
                "mode": mode,
            })
            while len(self._window) > self.window_size:
                self._window.popleft()


    def _extract_features(self, window_samples: list) -> np.ndarray:
        discrepancies = np.array([s["discrepancy"] for s in window_samples])
        abs_d = np.abs(discrepancies)

        pos_abs = abs_d[:, :3]
        rot_abs = abs_d[:, 3:]
        pos_norm = np.linalg.norm(discrepancies[:, :3], axis=1)

        features = np.concatenate([
            pos_abs.mean(axis=0),
            pos_abs.max(axis=0),
            rot_abs.mean(axis=0),
            rot_abs.max(axis=0),
            [pos_norm.mean()],
            [pos_norm.max()],
        ])
        return features


    def _predict_loop(self) -> None:
        while True:
            time.sleep(self.publish_interval)
            try:
                with self._lock:
                    window_copy = list(self._window)

                if len(window_copy) < self.window_size // 2:
                    continue

                running_samples = [s for s in window_copy if s["mode"] == RobotMode.ROBOT_MODE_RUNNING]
                samples_for_features = running_samples if running_samples else window_copy

                features = self._extract_features(samples_for_features)
                pos_discrepancy_mean = features[12]

                if self._model is not None:
                    try:
                        wear_level = float(self._model.predict(features.reshape(1, -1))[0])
                        wear_level = float(np.clip(wear_level, 0.0, 1.0))
                    except Exception as e:
                        self._l.warning(f"Model prediction failed: {e}")
                        wear_level = None
                else:
                    wear_level = None

                with self._lock:
                    self._latest_wear = wear_level if wear_level is not None else 0.0
                    self._latest_tcp_discrepancy = pos_discrepancy_mean

                self._publish(features, wear_level, pos_discrepancy_mean)

            except Exception as e:
                self._l.warning(f"Predict loop error: {e}", exc_info=e)


    def _publish(self, features: np.ndarray, wear_level, pos_discrepancy_mean: float,):
        ts = datetime.now(timezone.utc).isoformat()
        model_available = self._model is not None

        msg = {
            "timestamp": ts,
            "model_available": model_available,
            "predicted_wear_level": round(wear_level, 4) if wear_level is not None else None,
            "tcp_pos_discrepancy_mean_m": round(float(pos_discrepancy_mean), 6),
            "tcp_pos_discrepancy_max_m": round(float(features[3:6].max()), 6),
            "features": [round(float(f), 6) for f in features],
        }

        self.rabbitmq_publisher.send_message(ROUTING_KEY_WEAR, msg)

        fields = {
            "tcp_pos_discrepancy_mean_m": float(pos_discrepancy_mean),
            "tcp_pos_discrepancy_max_m": float(features[3:6].max()),
            "model_available": int(model_available),
        }
        if wear_level is not None:
            fields["predicted_wear_level"] = float(wear_level)

        try:
            self.write_api.write(
                self.influx_db_bucket,
                self.influx_db_org,
                {
                    "measurement": "wear_prediction",
                    "tags": {"source": "wear_prediction_service"},
                    "time": ts,
                    "fields": fields,
                },
            )
        except Exception as e:
            self._l.warning(f"InfluxDB write failed: {e}")

        wear_str = f"{wear_level:.3f}" if wear_level is not None else "N/A"
        self._l.info(f"Wear estimate | wear={wear_str} | TCP discrepancy mean={pos_discrepancy_mean*1000:.2f} mm")


    def start_serving(self) -> None:
        threading.Thread(target=self._predict_loop, daemon=True).start()
        self._l.info(
            f"WearPredictionService started "
            f"(window={self.window_size} samples, "
            f"publish_interval={self.publish_interval}s, "
            f"model={'loaded' if self._model else 'not found - raw mode'})."
        )
        try:
            self.rabbitmq_consumer.start_consuming()
        except KeyboardInterrupt:
            self.cleanup()

    def cleanup(self) -> None:
        self.rabbitmq_consumer.close()
        self.rabbitmq_publisher.close()
