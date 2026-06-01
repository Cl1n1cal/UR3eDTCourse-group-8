from abc import abstractmethod
import math
import mstlo_python as mstlo
from typing import List, Tuple, Union
from communication.protocol import MonitoringMsgKeys, MonitoringMsgTypes, RobotArmStateKeys, RobotMode
import numpy as np
import time

from utils.calculation_functions import milliseconds_to_seconds

class UR3eMonitor:
    """Abstract base for UR3e monitor classes.

    Subclasses should implement:
    - `type`: the monitoring message type string to use when sending results
    - `formula`: the temporal logic formula string
    - `initialize_monitor` / `_initialize_monitor`: build the `mstlo.Monitor`
    - `compute_robustness`: accept sample data and return monitor verdicts

    The `latest()` helper returns the last verdict value or None.
    """

    @property
    @abstractmethod
    def type(self) -> str:
        pass

    @property
    @abstractmethod
    def formula(self) -> str:
        pass

    @abstractmethod
    def initialize_monitor(self) -> mstlo.Monitor:
        pass

    @abstractmethod
    def compute_robustness(self, *args) -> List[Tuple[float, Union[bool, float, Tuple[float, float]]]] | None:
        pass

    @staticmethod
    def latest(verdicts : List[Tuple[float, Union[bool, float, Tuple[float, float]]]]):
        """Return the value of the last verdict, or None if no verdicts."""
        if not verdicts:
            return None
        return verdicts[-1][1]

    def _skip_if_not_new_timestamp(self, timestamp: float) -> bool:
        last_timestamp = getattr(self, "_last_timestamp", None)
        if last_timestamp is not None and timestamp <= last_timestamp:
            return True
        self._last_timestamp = timestamp
        return False

class LatencyMonitor(UR3eMonitor):
    """Latency monitor that checks time alignment between the monitor and a sample.

    The monitor exposes a single `time_diff` signal (absolute difference between
    the remote sample timestamp and the adjusted local timestamp). The class
    holds a configured `$max_latency` variable used by the formula.
    """
    def __init__(self, max_latency: float, sample_delay: float, s_type: str):
        """Initialize the latency monitor. s_type can be "mockup" or "simulation"."""
        self.max_latency = max_latency
        self.sample_delay = sample_delay
        self.s_type = s_type
        self._last_timestamp: float | None = None
        self._monitor = self._initialize_monitor()

    @property
    def type(self) -> str:
        if self.s_type == "mockup":
            return MonitoringMsgTypes.MOCKUP_OFFLINE
        elif self.s_type == "simulation":
            return MonitoringMsgTypes.SIMULATION_OFFLINE
        else:
            return "Unknown Latency Monitor"

    

    @property
    def formula(self) -> str:
        return "(time_diff < $max_latency)"

    def _initialize_monitor(self) -> mstlo.Monitor:
        vars = mstlo.Variables()
        vars.set("max_latency", self.max_latency)
        spec = mstlo.parse_formula(self.formula)
        return mstlo.Monitor(
            formula=spec, semantics="DelayedQuantitative", variables=vars
        )

    def compute_robustness(self, *args) -> List[Tuple[float, Union[bool, float, Tuple[float, float]]]] | None:
        """Compute robustness for a latency sample.

        Expects `(sample_data, nan, time_stamp)` where `sample_data` contains a
        remote `time_stamp`. Updates the `time_diff` signal and returns
        `mstlo` verdicts.
        """
        time_stamp = args[2]

        if self.s_type == "mockup":
            sample_data = args[0]
        elif self.s_type == "simulation":
            sample_data = args[1]
        else:
            return None

        if sample_data is None:
            return None

        sample_time = sample_data.get("time_stamp")
        if sample_time is None:
            return None

        # Calculate the difference in time stamps for the mockup and the monitor service
        adj_time = time_stamp - (milliseconds_to_seconds(self.sample_delay))
        if self._skip_if_not_new_timestamp(adj_time):
            return None
        time_diff = np.absolute(sample_time - adj_time)

        output = self._monitor.update(
            signal="time_diff", value=time_diff, timestamp=adj_time
        )

        return output.verdicts()
    
class VelocityMonitor(UR3eMonitor):
    """Monitor that flags when the maximum joint velocity is exceeded.

    The monitor expects a single `qd` signal which should be set to the
    largest joint velocity observed in the sample. The variable `$max_velocity`
    is kept in sync with the sample's reported joint max speed.
    """
    def __init__(self, max_velocity, window_seconds: float = 0.0):
        self.max_velocity = max_velocity
        self.window_seconds = max(0.0, float(window_seconds))
        self._monitor = self._initialize_monitor()
        self._last_timestamp = -1.0 

    @property
    def type(self) -> str:
        return MonitoringMsgTypes.MAX_VELOCITY_EXCEEDED

    @property
    def formula(self) -> str:
        return f"G[0,{self.window_seconds}](qd < $max_velocity)"

    def _initialize_monitor(self) -> mstlo.Monitor:
        vars = mstlo.Variables()
        vars.set("max_velocity", self.max_velocity)
        spec = mstlo.parse_formula(self.formula)
        return mstlo.Monitor(
            formula=spec, semantics="DelayedQuantitative", variables=vars
        )

    def compute_robustness(self, *args) -> List[Tuple[float, Union[bool, float, Tuple[float, float]]]] | None:
        """Compute robustness using the provided `mockup_data`.

        Expects `mockup_data` with keys `qd_actual_<i>` and
        `joint_max_speed`. Uses the largest `qd_actual` as the `qd` signal.
        """
        mockup_data = args[0]

        if mockup_data is None:
            return None

        max_velocity = mockup_data.get(RobotArmStateKeys.JOINT_MAX_SPEED)
        time = mockup_data.get("time_stamp")

        if max_velocity is None or time is None:
            return None

        if self._skip_if_not_new_timestamp(time):
            return None

        # Get the current velocity of each joint
        qd_list = []
        for i in range(6):
            val = mockup_data.get(f"qd_actual_{i}")
            if val is None: 
                return None
            qd_list.append(val)
        
        # Find the joint with the largest velocity
        largest_qd = max(qd_list)

        # Update the max velocity variable
        self._monitor.get_variables().set("max_velocity", max_velocity)

        # Update the qd signal in the monitor
        output = self._monitor.update('qd', largest_qd, time)

        return output.verdicts()

class AccelerationMonitor(UR3eMonitor):
    """Monitor for joint acceleration limits.

    The monitor computes per-step accelerations by differencing consecutive
    `qd_actual_<i>` values and dividing by the elapsed time between samples.
    It retains the last `mockup_data` to compute deltas on the next update.
    """
    def __init__(self, max_acceleration, window_seconds: float = 0.0):
        self.max_acceleration = max_acceleration
        self.window_seconds = max(0.0, float(window_seconds))
        self._monitor = self._initialize_monitor()
        self.old_mockup_data = None

    @property
    def type(self) -> str:
        return MonitoringMsgTypes.MAX_ACCELERATION_EXCEEDED

    @property
    def formula(self) -> str:
        return f"G[0,{self.window_seconds}](qdd < $max_acceleration)"

    def _initialize_monitor(self) -> mstlo.Monitor:
        vars = mstlo.Variables()
        vars.set("max_acceleration", self.max_acceleration)
        spec = mstlo.parse_formula(self.formula)
        return mstlo.Monitor(
            formula=spec, semantics="DelayedQuantitative", variables=vars
        )

    def compute_robustness(self, *args) -> List[Tuple[float, Union[bool, float, Tuple[float, float]]]] | None:
        """Compute acceleration robustness using `mockup_data`.

        On the first call the previous sample is not available and the method
        stores the sample then returns `None`. On subsequent calls it computes
        the largest acceleration across joints and updates the monitor's `qdd`
        signal.
        """
        mockup_data = args[0]
        time_new = mockup_data["time_stamp"] if mockup_data is not None else None
        if mockup_data is None or time_new is None:
            return None

        if self._skip_if_not_new_timestamp(time_new):
            return None
        # Here, the last velocity is also checked since in the first loop it will be None
        if self.old_mockup_data is None:
            #store old values
            self.old_mockup_data = mockup_data.copy()
            return None
        
        # Get the max velocity and the time from the newest mockup_data
        max_acceleration = mockup_data[RobotArmStateKeys.JOINT_MAX_ACCELERATION]

        if max_acceleration is None:
            self.old_mockup_data = mockup_data.copy()
            return None

        # Get the time from the old mockup_data
        time_old = self.old_mockup_data["time_stamp"] 

        # Acceleration is change in velocity over change in time: a = v_1 - v_0 / t_1 - t_0
        # Create a list of the difference in velocities
        qd_diff_list = []
        for i in range(6):
            v_new = mockup_data[f"qd_actual_{i}"]
            v_old = self.old_mockup_data[f"qd_actual_{i}"]
            if v_new is None or v_old is None:
                self.old_mockup_data = mockup_data.copy()
                return None
            qd_diff_list.append(np.abs(v_new - v_old))

        # Find the joint with the largest velocity difference (will have largest acceleration)
        max_qd_diff = max(qd_diff_list)

        # Set a constant value if the joint is not moving (max qd diff very small), otherwise we get 'nan' from monitor
        if max_qd_diff < 0.00001:
            max_qd_diff = 0.00000001

        # Compute the differnce in time
        time_diff = time_new - time_old

        # Prevent divission by 0
        if time_diff <= 0.0:
            print("Warning: time difference is zero in AccelerationMonitor. Returning None.")
            return None

        # Compute the acceleration for that joint
        largest_qdd = max_qd_diff / time_diff

        self._monitor.get_variables().set("max_acceleration", max_acceleration)

        output = self._monitor.update('qdd', largest_qdd, time_new)

        self.old_mockup_data = mockup_data.copy()

        return output.verdicts()
    
class StuckJointMonitor(UR3eMonitor):
    def __init__(self, joint_index: int, qd_threshold: float = 0.01, q_diff_threshold: float = 0.1, window_seconds: float = 0.0):
        """Monitor that detects a stuck joint by checking low joint velocity and
        a large difference between target and actual joint angle.

        Args:
            joint_index: index of the joint (0-5)
            qd_threshold: minimum absolute joint velocity considered as movement
            q_diff_threshold: minimum position error considered as stuck
        """
        self.joint_index = int(joint_index)
        self.qd_threshold = qd_threshold
        self.q_diff_threshold = q_diff_threshold
        self.window_seconds = max(0.0, float(window_seconds))
        self._last_timestamp: float | None = None
        self._monitor = self._initialize_monitor()
        self.old_mockup_data = None

    @property
    def type(self) -> str:
        return getattr(MonitoringMsgTypes, f"STUCK_JOINT_{self.joint_index}")

    @property
    def formula(self) -> str:
        # The monitor expects three signals: 'qd', 'q_diff', and 'robot_mode'.
        # robot_mode is encoded as 1 for Running and 0 for Idle so the formula
        # stays true while the robot is idle and only checks for sticking when
        # the robot is running.
        return f"G[0,{self.window_seconds}](robot_mode < 0.5 || qd > $qd_threshold || q_diff < $q_diff_threshold)"

    def _initialize_monitor(self) -> mstlo.Monitor:
        variables = mstlo.Variables()
        variables.set("qd_threshold", self.qd_threshold)
        variables.set("q_diff_threshold", self.q_diff_threshold)
        spec = mstlo.parse_formula(self.formula)
        return mstlo.Monitor(formula=spec, semantics="DelayedQuantitative", variables=variables)

    def compute_robustness(self, *args) -> List[Tuple[float, Union[bool, float, Tuple[float, float]]]] | None:
        """Expect a single argument: mockup_data dict containing keys for this joint.

        The method produces a batch update with the signals the monitor expects
        and returns the monitor verdicts.
        """
        mockup_data = args[0]
        if mockup_data is None:
            return None

        if self.old_mockup_data is None:
            self.old_mockup_data = mockup_data.copy()
            return None

        time_stamp = mockup_data["time_stamp"]
        if self._skip_if_not_new_timestamp(time_stamp):
            return None
        time_diff = time_stamp - self.old_mockup_data["time_stamp"]
        if time_diff <= 0.0:
            return None

        qd = abs(mockup_data[f"q_actual_{self.joint_index}"] - self.old_mockup_data[f"q_actual_{self.joint_index}"]) / time_diff
        q_diff = abs(mockup_data[f"q_target_{self.joint_index}"] - mockup_data[f"q_actual_{self.joint_index}"])
        robot_mode = 1.0 if mockup_data.get(RobotArmStateKeys.ROBOT_MODE) == RobotMode.ROBOT_MODE_RUNNING else 0.0

        batch = {
            "qd": [(qd, time_stamp)],
            "q_diff": [(q_diff, time_stamp)],
            "robot_mode": [(robot_mode, time_stamp)],
        }

        output = self._monitor.update_batch(batch)
        self.old_mockup_data = mockup_data.copy()
        return output.verdicts()


class MismatchMonitor(UR3eMonitor):
    def __init__(self, diff_type: str, max_error: float, window_seconds: float = 0.0):
        """Generic monitor for comparing mockup <-> simulation.

        Args:
            diff_type: 'tcp' or 'q' to indicate pose or joint-space mismatch
            max_error: upper bound for acceptable difference
        """
        self.diff_type = diff_type
        self.max_error = max_error
        self.window_seconds = max(0.0, float(window_seconds))
        self._last_timestamp: float | None = None
        self._monitor = self._initialize_monitor()

    @property
    def type(self) -> str:
        if self.diff_type == "tcp":
            return MonitoringMsgTypes.TCP_MISSMATCH
        return MonitoringMsgTypes.Q_MISSMATCH

    @property
    def formula(self) -> str:
        return f"G[0,{self.window_seconds}](q_diff < $max_error)"

    def _initialize_monitor(self) -> mstlo.Monitor:
        variables = mstlo.Variables()
        variables.set("max_error", self.max_error)
        spec = mstlo.parse_formula(self.formula)
        return mstlo.Monitor(formula=spec, semantics="DelayedQuantitative", variables=variables)

    def compute_robustness(self, *args) -> List[Tuple[float, Union[bool, float, Tuple[float, float]]]] | None:
        """Expects (mockup_data, sim_data). Computes a Euclidean difference and updates the monitor."""
        mockup_data = args[0]
        sim_data = args[1]
        if mockup_data is None or sim_data is None:
            return None

        if self.diff_type == "tcp":
            keys = [f"tcp_pose_{i}" for i in range(6)]
            timestamp = mockup_data["time_stamp"]
        else:
            keys = [f"q_actual_{i}" for i in range(6)]
            timestamp = max(mockup_data["time_stamp"], sim_data["time_stamp"])

        for k in keys:
            if mockup_data.get(k) is None or sim_data.get(k) is None:
                return None

        error = sum((mockup_data[k] - sim_data[k]) ** 2 for k in keys) ** 0.5
        if self._skip_if_not_new_timestamp(timestamp):
            return None
        output = self._monitor.update(signal="q_diff", value=abs(error), timestamp=timestamp)
        return output.verdicts()


class WearPredictionMonitor(UR3eMonitor):
    """Monitor that checks the predicted wear level published by the wear prediction service.

    The measurement written by the wear prediction service is `wear_prediction` and
    contains the field `predicted_wear_level`. This monitor expects the monitoring
    loop to pass that entry as an extra argument to `compute_robustness` (args[3]).
    """
    def __init__(self, max_wear: float, window_seconds: float = 0.0):
        self.max_wear = max_wear
        self.window_seconds = max(0.0, float(window_seconds))
        self._last_timestamp: float | None = None
        self._monitor = self._initialize_monitor()

    @property
    def type(self) -> str:
        return MonitoringMsgTypes.WEAR_PREDICTION

    @property
    def formula(self) -> str:
        # Expects a signal named 'wear' to be updated with the predicted wear level
        return f"G[0,{self.window_seconds}](wear < $max_wear)"

    def _initialize_monitor(self) -> mstlo.Monitor:
        vars = mstlo.Variables()
        vars.set("max_wear", self.max_wear)
        spec = mstlo.parse_formula(self.formula)
        return mstlo.Monitor(formula=spec, semantics="DelayedQuantitative", variables=vars)

    def compute_robustness(self, *args) -> List[Tuple[float, Union[bool, float, Tuple[float, float]]]] | None:
        """Expects (mockup_data, sim_data, timestamp, wear_data).

        If `wear_data` is not provided or contains no `predicted_wear_level`, returns None.
        """
        wear_data = None
        if len(args) >= 4:
            wear_data = args[3]

        if wear_data is None:
            return None

        wear_val = wear_data.get("predicted_wear_level")
        if wear_val is None:
            return None

        timestamp = wear_data.get("time_stamp", time.time())
        if self._skip_if_not_new_timestamp(timestamp):
            return None

        # Update the variable and signal
        self._monitor.get_variables().set("max_wear", self.max_wear)
        output = self._monitor.update(signal="wear", value=float(wear_val), timestamp=timestamp)
        self._last_timestamp = timestamp
        return output.verdicts()

class JointRotationMonitor(UR3eMonitor):
    def __init__(self, joint_index: int, rotation_threshold: float = 0.0):
        self.joint_index = int(joint_index)
        self.rotation_threshold = float(rotation_threshold)
        self._accumulated_rotations: float = 0.0
        self._prev_q: float | None = None
        self._last_timestamp: float | None = None
        self._monitor = self._initialize_monitor()

    @property
    def name(self) -> str:
        return f"joint_{self.joint_index}_rotations"

    @property
    def type(self) -> str:
        return getattr(MonitoringMsgTypes, f"JOINT_ROTATION_THRESHOLD_{self.joint_index}")

    @property
    def monitor(self) -> mstlo.Monitor:
        return self._monitor

    @property
    def formula(self) -> str:
        return "(rotations < $threshold)"

    def _initialize_monitor(self) -> mstlo.Monitor:
        variables = mstlo.Variables()
        variables.set("threshold", self.rotation_threshold if self.rotation_threshold > 0 else 1e9)
        spec = mstlo.parse_formula(self.formula)
        return mstlo.Monitor(formula=spec, semantics="DelayedQuantitative", variables=variables)

    def set_accumulated_rotations(self, rotations: float) -> None:
        """Inject a previously persisted rotation count (e.g. loaded from DB on startup)."""
        self._accumulated_rotations = float(rotations)

    def get_accumulated_rotations(self) -> float:
        return self._accumulated_rotations

    def compute_robustness(self, *args) -> List[Tuple[float, Union[bool, float, Tuple[float, float]]]] | None:
        mockup_data = args[0]
        time_stamp = args[2] if len(args) > 2 else None

        if mockup_data is None:
            return None

        q_current = mockup_data.get(f"q_actual_{self.joint_index}")
        if q_current is None:
            return None

        ts = time_stamp if time_stamp is not None else mockup_data.get("time_stamp", 0.0)

        if self._prev_q is not None:
            delta_rad = abs(q_current - self._prev_q)
            self._accumulated_rotations += delta_rad / (2.0 * math.pi)

        self._prev_q = q_current

        if self._skip_if_not_new_timestamp(ts):
            return None

        if self.rotation_threshold > 0:
            self._monitor.get_variables().set("threshold", self.rotation_threshold)

        output = self._monitor.update("rotations", self._accumulated_rotations, ts)
        return output.verdicts()
