from abc import abstractmethod
import mstlo_python as mstlo
from typing import List, Tuple, Union
from communication.protocol import MonitoringMsgKeys, MonitoringMsgTypes, RobotArmStateKeys, RobotMode
import numpy as np

from utils.calculation_functions import milliseconds_to_seconds

class UR3eMonitor:
    """Abstract base for UR3e monitor classes.

    Subclasses should implement:
    - `name`: mapping to a `MonitoringMsgTypes` value
    - `monitor`: the underlying `mstlo.Monitor` instance
    - `formula`: the temporal logic formula string
    - `initialize_monitor` / `_initialize_monitor`: build the `mstlo.Monitor`
    - `compute_robustness`: accept sample data and return monitor verdicts

    The `latest()` helper returns the last verdict value or None.
    """

    @property
    @abstractmethod
    def name(self) -> str:
        pass
    
    @property
    @abstractmethod
    def type(self) -> str:
        pass

    @property
    @abstractmethod
    def monitor(self) -> mstlo.Monitor:
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
        self._monitor = self._initialize_monitor()

    @property
    def name(self) -> str:
        if self.s_type == "mockup":
            return "mockup_latency"
        elif self.s_type == "simulation":
            return "simulation_latency"
        else:
            return "Unknown Latency Monitor"
        
    @property
    def type(self) -> str:
        if self.s_type == "mockup":
            return MonitoringMsgTypes.MOCKUP_OFFLINE
        elif self.s_type == "simulation":
            return MonitoringMsgTypes.SIMULATION_OFFLINE
        else:
            return "Unknown Latency Monitor"

    @property
    def monitor(self) -> mstlo.Monitor:
        return self._monitor

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

    def compute_robustness(self, *args) -> List[Tuple[float, Union[bool, float, Tuple[float, float]]]]:
        """Compute robustness for a latency sample.

        Expects `(sample_data, nan, time_stamp)` where `sample_data` contains a
        remote `time_stamp`. Updates the `time_diff` signal and returns
        `mstlo` verdicts.
        """
        time_stamp = args[2]
        sample_data = args[0]
        # Calculate the difference in time stamps for the mockup and the monitor service
        adj_time = time_stamp - (milliseconds_to_seconds(self.sample_delay))
        sample_time = sample_data["time_stamp"]
        time_diff = np.absolute(sample_time - adj_time)

        output = self.monitor.update(
            signal="time_diff", value=time_diff, timestamp=adj_time
        )

        return output.verdicts()
    
class VelocityMonitor(UR3eMonitor):
    """Monitor that flags when the maximum joint velocity is exceeded.

    The monitor expects a single `qd` signal which should be set to the
    largest joint velocity observed in the sample. The variable `$max_velocity`
    is kept in sync with the sample's reported joint max speed.
    """
    def __init__(self, max_velocity):
        self.max_velocity = max_velocity
        self._monitor = self._initialize_monitor()

    @property
    def name(self) -> str:
        return "mockup_velocity"

    @property
    def type(self) -> str:
        return MonitoringMsgTypes.MAX_VELOCITY_EXCEEDED

    @property
    def monitor(self) -> mstlo.Monitor:
        return self._monitor

    @property
    def formula(self) -> str:
        return "(qd < $max_velocity)"

    def _initialize_monitor(self) -> mstlo.Monitor:
        vars = mstlo.Variables()
        vars.set("max_velocity", self.max_velocity)
        spec = mstlo.parse_formula(self.formula)
        return mstlo.Monitor(
            formula=spec, semantics="DelayedQuantitative", variables=vars
        )

    def compute_robustness(self, *args) -> List[Tuple[float, Union[bool, float, Tuple[float, float]]]]:
        """Compute robustness using the provided `mockup_data`.

        Expects `mockup_data` with keys `qd_actual_<i>` and
        `joint_max_speed`. Uses the largest `qd_actual` as the `qd` signal.
        """
        mockup_data = args[0]
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
        self.monitor.get_variables().set("max_velocity", max_velocity)

        # Update the qd signal in the monitor
        output = self.monitor.update('qd', largest_qd, time)

        return output.verdicts()

class AccelerationMonitor(UR3eMonitor):
    """Monitor for joint acceleration limits.

    The monitor computes per-step accelerations by differencing consecutive
    `qd_actual_<i>` values and dividing by the elapsed time between samples.
    It retains the last `mockup_data` to compute deltas on the next update.
    """
    def __init__(self, max_acceleration):
        self.max_acceleration = max_acceleration
        self._monitor = self._initialize_monitor()
        self.old_mockup_data = None

    @property
    def name(self) -> str:
        return "mockup_acceleration"

    @property
    def type(self) -> str:
        return MonitoringMsgTypes.MAX_ACCELERATION_EXCEEDED

    @property
    def monitor(self) -> mstlo.Monitor:
        return self._monitor

    @property
    def formula(self) -> str:
        return "(qdd < $max_acceleration)"

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
        # Here, the last velocity is also checked since in the first loop it will be None
        if self.old_mockup_data is None:
            #store old values
            self.old_mockup_data = mockup_data.copy()
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
            print("Warning: time difference is zero in AccelerationMonitor. Returning None.")
            return None

        # Compute the acceleration for that joint
        largest_qdd = max_qd_diff / time_diff

        self.monitor.get_variables().set("max_acceleration", max_acceleration)

        output = self.monitor.update('qdd', largest_qdd, time_new)

        self.old_mockup_data = mockup_data.copy()

        return output.verdicts()
    
class StuckJointMonitor(UR3eMonitor):
    def __init__(self, joint_index: int, qd_threshold: float = 0.01, q_diff_threshold: float = 0.1):
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
        self._monitor = self._initialize_monitor()
        self.old_mockup_data = None

    @property
    def name(self) -> str:
        return "stuck_joint_" + str(self.joint_index)

    @property
    def type(self) -> str:
        return getattr(MonitoringMsgTypes, f"STUCK_JOINT_{self.joint_index}")

    @property
    def monitor(self) -> mstlo.Monitor:
        return self._monitor

    @property
    def formula(self) -> str:
        # The monitor expects three signals: 'qd', 'q_diff', and 'robot_mode'.
        # robot_mode is encoded as 1 for Running and 0 for Idle so the formula
        # stays true while the robot is idle and only checks for sticking when
        # the robot is running.
        return "(robot_mode < 0.5 || qd > $qd_threshold || q_diff < $q_diff_threshold)"

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
        time_diff = time_stamp - self.old_mockup_data["time_stamp"]
        if time_diff == 0.0:
            return None

        qd = abs(mockup_data[f"q_actual_{self.joint_index}"] - self.old_mockup_data[f"q_actual_{self.joint_index}"]) / time_diff
        q_diff = abs(mockup_data[f"q_target_{self.joint_index}"] - mockup_data[f"q_actual_{self.joint_index}"])
        robot_mode = 1.0 if mockup_data.get(RobotArmStateKeys.ROBOT_MODE) == RobotMode.ROBOT_MODE_RUNNING else 0.0

        batch = {
            "qd": [(qd, time_stamp)],
            "q_diff": [(q_diff, time_stamp)],
            "robot_mode": [(robot_mode, time_stamp)],
        }

        output = self.monitor.update_batch(batch)
        self.old_mockup_data = mockup_data.copy()
        return output.verdicts()


class MismatchMonitor(UR3eMonitor):
    def __init__(self, diff_type: str, max_error: float):
        """Generic monitor for comparing mockup <-> simulation.

        Args:
            diff_type: 'tcp' or 'q' to indicate pose or joint-space mismatch
            max_error: upper bound for acceptable difference
        """
        self.diff_type = diff_type
        self.max_error = max_error
        self._monitor = self._initialize_monitor()

    @property
    def name(self) -> str:
        if self.diff_type == "tcp":
            return "tcp_mismatch"
        return "q_mismatch"

    @property
    def type(self) -> str:
        if self.diff_type == "tcp":
            return MonitoringMsgTypes.TCP_MISSMATCH
        return MonitoringMsgTypes.Q_MISSMATCH

    @property
    def monitor(self) -> mstlo.Monitor:
        return self._monitor

    @property
    def formula(self) -> str:
        return "(q_diff < $max_error)"

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
            error = sum((mockup_data[f"tcp_pose_{i}"] - sim_data[f"tcp_pose_{i}"])**2 for i in range(6))**0.5
            timestamp = mockup_data["time_stamp"]
        else:
            error = sum((mockup_data[f"q_actual_{i}"] - sim_data[f"q_actual_{i}"])**2 for i in range(6))**0.5
            timestamp = max(mockup_data["time_stamp"], sim_data["time_stamp"])

        output = self.monitor.update(signal="q_diff", value=abs(error), timestamp=timestamp)
        return output.verdicts()
