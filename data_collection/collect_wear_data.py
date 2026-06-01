import sys
import os
import time
import json
import csv
import threading
import numpy as np
from collections import deque
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from communication.factory import RabbitMQFactory
from communication.protocol import RobotArmStateKeys, RobotMode, ROUTING_KEY_STATE, ROUTING_KEY_CTRL, CtrlMsgKeys, CtrlMsgFields, FaultTypes
from models.robot_model import create_robot
from utils.calculation_functions import se3_to_pos_rpy
from utils.configuration import load_config

OUTPUT_PATH = Path(__file__).parent / "wear_dataset.csv"

WEAR_LEVELS = [round(i * 0.1, 1) for i in range(11)]
N_TRAJECTORIES = 200
WEAR_DURATION_S = 30
TRAJECTORY_DURATION_S = 8
SAMPLE_COLLECTION_S = 6
INTER_TRAJECTORY_PAUSE_S = 1.5

RNG = np.random.default_rng(seed=42)

JOINT_LIMITS_LOW  = np.array([-2*np.pi, -2*np.pi, -np.pi, -2*np.pi, -2*np.pi, -2*np.pi])
JOINT_LIMITS_HIGH = np.array([ 2*np.pi, 2*np.pi, np.pi, 2*np.pi, 2*np.pi, 2*np.pi])

VELOCITY = 60
ACCELERATION = 80

NUM_JOINTS = 6
FEATURE_NAMES = [
    "mean_dx", "mean_dy", "mean_dz",
    "max_dx",  "max_dy",  "max_dz",
    "mean_drx","mean_dry","mean_drz",
    "max_drx", "max_dry", "max_drz",
    "mean_pos_error_m",
    "max_pos_error_m",
]


class StateListener:
    def __init__(self, robot):
        self.robot = robot
        self._samples: deque = deque()
        self._collecting = threading.Event()
        self._lock = threading.Lock()
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()

    def setup(self):
        self.rabbitmq.connect_to_server()
        self.rabbitmq.subscribe(routing_key=ROUTING_KEY_STATE, on_message_callback=self._on_message)

    def _on_message(self, ch, method, properties, message: dict):
        if not self._collecting.is_set():
            return
        q_actual = message.get(RobotArmStateKeys.Q_ACTUAL)
        tcp_pose = message.get(RobotArmStateKeys.TCP_POSE)
        mode = message.get(RobotArmStateKeys.ROBOT_MODE)

        if q_actual is None or tcp_pose is None:
            return
        if len(q_actual) != NUM_JOINTS or len(tcp_pose) != 6:
            return

        if mode != RobotMode.ROBOT_MODE_RUNNING:
            return

        try:
            model_tcp = se3_to_pos_rpy(self.robot.fkine(np.array(q_actual)))
        except Exception:
            return

        discrepancy = np.array(tcp_pose) - model_tcp

        with self._lock:
            self._samples.append(discrepancy)

    def start_collecting(self):
        with self._lock:
            self._samples.clear()
        self._collecting.set()

    def stop_collecting(self):
        self._collecting.clear()

    def get_samples(self) -> list:
        with self._lock:
            return list(self._samples)

    def start_background(self):
        t = threading.Thread(target=self.rabbitmq.start_consuming, daemon=True)
        t.start()


def extract_features(discrepancy_samples: list) -> np.ndarray | None:
    if len(discrepancy_samples) < 3:
        return None

    d = np.array(discrepancy_samples)
    abs_d = np.abs(d)
    pos_abs = abs_d[:, :3]
    rot_abs = abs_d[:, 3:]
    pos_norm = np.linalg.norm(d[:, :3], axis=1)

    features = np.concatenate([
        pos_abs.mean(axis=0),
        pos_abs.max(axis=0),
        rot_abs.mean(axis=0),
        rot_abs.max(axis=0),
        [pos_norm.mean()],
        [pos_norm.max()],
    ])
    return features


def send_inject_wear(publisher, wear_level: float, duration: float):
    msg = {
        CtrlMsgKeys.TYPE: CtrlMsgFields.INJECT_FAULT,
        CtrlMsgKeys.FAULT_TYPE: FaultTypes.WEAR,
        CtrlMsgKeys.FAULT_VALUE: wear_level,
        CtrlMsgKeys.JOINTS: list(range(NUM_JOINTS)),
        CtrlMsgKeys.DURATION: duration,
    }
    publisher.send_message(ROUTING_KEY_CTRL, msg)
    print(f"Injected wear={wear_level} for {duration}s")


def send_load_program(publisher, q_end: list, velocity: float, acceleration: float):
    msg = {
        CtrlMsgKeys.TYPE: CtrlMsgFields.LOAD_PROGRAM,
        CtrlMsgKeys.JOINT_POSITIONS: [q_end],
        CtrlMsgKeys.MAX_VELOCITY: velocity,
        CtrlMsgKeys.ACCELERATION: acceleration,
    }
    publisher.send_message(ROUTING_KEY_CTRL, msg)


def send_play(publisher):
    publisher.send_message(ROUTING_KEY_CTRL, {CtrlMsgKeys.TYPE: CtrlMsgFields.PLAY})


def send_stop(publisher):
    publisher.send_message(ROUTING_KEY_CTRL, {CtrlMsgKeys.TYPE: CtrlMsgFields.STOP})


def main():
    config = load_config("startup/startup.conf")
    dh = config["digital_twin"]["robot_model"]["dh_parameters"]
    robot = create_robot(
        d=list(dh["d"]),
        a=list(dh["a"]),
        alpha=list(dh["alpha"]),
    )

    listener = StateListener(robot)
    listener.setup()
    listener.start_background()

    publisher = RabbitMQFactory.create_rabbitmq()
    publisher.connect_to_server()

    print("Wear data collection starting …")
    print(f"Wear levels: {WEAR_LEVELS}")
    print(f"Trajectories: {N_TRAJECTORIES} per level")
    print(f"Output: {OUTPUT_PATH}")
    print()
    time.sleep(2.0)

    rows = []

    for wear_level in WEAR_LEVELS:
        print(f"\n=== Wear level {wear_level:.1f} ===")

        wear_window = WEAR_DURATION_S + N_TRAJECTORIES * (TRAJECTORY_DURATION_S + INTER_TRAJECTORY_PAUSE_S)
        send_inject_wear(publisher, wear_level, wear_window)
        time.sleep(0.5)

        for traj_idx in range(N_TRAJECTORIES):
            q_target = RNG.uniform(JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH).tolist()

            print(f"  Trajectory {traj_idx+1}/{N_TRAJECTORIES} … ", end="", flush=True)

            send_load_program(publisher, q_target, VELOCITY, ACCELERATION)
            time.sleep(0.1)
            send_play(publisher)

            listener.start_collecting()
            time.sleep(SAMPLE_COLLECTION_S)
            listener.stop_collecting()

            time.sleep(TRAJECTORY_DURATION_S - SAMPLE_COLLECTION_S + INTER_TRAJECTORY_PAUSE_S)
            send_stop(publisher)

            samples = listener.get_samples()
            features = extract_features(samples)

            if features is None:
                print(f"SKIPPED (only {len(samples)} samples)")
                continue

            row = {name: features[i] for i, name in enumerate(FEATURE_NAMES)}
            row["wear_level"] = wear_level
            row["n_samples"] = len(samples)
            rows.append(row)
            print(f"OK  ({len(samples)} samples, mean_pos_err={features[12]*1000:.2f} mm)")

        time.sleep(1.0)

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = FEATURE_NAMES + ["wear_level", "n_samples"]
    with open(OUTPUT_PATH, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nCollection complete. {len(rows)} samples written to {OUTPUT_PATH}")
    publisher.close()


if __name__ == "__main__":
    main()
