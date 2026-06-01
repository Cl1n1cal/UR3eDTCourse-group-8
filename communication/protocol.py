import json

ENCODING = "ascii"

### ROUTING KEYS
ROUTING_KEY_STATE = "robotarm.pt.state"
ROUTING_KEY_MODEL_STATE = "robotarm.model.state"
ROUTING_KEY_PARTICLE = "robotarm.particle.state"
ROUTING_KEY_CTRL = "robotarm.ctrl"
ROUTING_KEY_RECORDER = "robotarm.recorder.#"
ROUTING_KEY_CALIBRATION = "robotarm.model.calibration"
ROUTING_KEY_MONITORING = "robotarm.monitoring"
ROUTING_KEY_ELECTRICITY = "robotarm.model.electricity"
ROUTING_KEY_JOINT_ROTATIONS = "robotarm.model.joint_rotations"
ROUTING_KEY_WEAR = "robotarm.model.wear"

### MESSAGES
class MonitoringMsgKeys:
    """Keys used in monitoring messages sent from the monitoring service."""

    TYPE = "type"
    ROBUSTNESS_VALUE = "robustness_value"
    ROBUSTNESS_UPPER_BOUND = "robustness_upper_bound"
    ROBUSTNESS_LOWER_BOUND = "robustness_lower_bound"
    TIMESTAMP = "timestamp"

class MonitoringMsgTypes:
    """Types of monitoring messages sent from the monitoring service."""
    STUCK_JOINT_0 = "stuck_joint_0"
    STUCK_JOINT_1 = "stuck_joint_1"
    STUCK_JOINT_2 = "stuck_joint_2"
    STUCK_JOINT_3 = "stuck_joint_3"
    STUCK_JOINT_4 = "stuck_joint_4"
    STUCK_JOINT_5 = "stuck_joint_5"

    WEAR_PREDICTION = "wear_prediction"

    TCP_MISSMATCH = "tcp_mismatch"
    Q_MISSMATCH = "q_mismatch"

    MAX_VELOCITY_EXCEEDED = "max_velocity_exceeded"
    MAX_ACCELERATION_EXCEEDED = "max_acceleration_exceeded"

    SIMULATION_OFFLINE = "simulation_offline"
    MOCKUP_OFFLINE = "mockup_offline"

    JOINT_ROTATION_THRESHOLD_0 = "Joint 0 Rotation Threshold"
    JOINT_ROTATION_THRESHOLD_1 = "Joint 1 Rotation Threshold"
    JOINT_ROTATION_THRESHOLD_2 = "Joint 2 Rotation Threshold"
    JOINT_ROTATION_THRESHOLD_3 = "Joint 3 Rotation Threshold"
    JOINT_ROTATION_THRESHOLD_4 = "Joint 4 Rotation Threshold"
    JOINT_ROTATION_THRESHOLD_5 = "Joint 5 Rotation Threshold"

class CtrlMsgFields:
    """Types of control messages that can be sent to the robot arm."""

    LOAD_PROGRAM = "load_program"
    PLAY = "play"
    PAUSE = "pause"
    STOP = "stop"
    INJECT_FAULT = "inject_fault"
    UNSTUCK_JOINT = "unstuck_joint"


class CtrlMsgKeys:
    """Keys used in control messages sent to the robot arm."""

    TYPE = "type"
    JOINT_POSITIONS = "joint_positions"
    MAX_VELOCITY = "max_velocity"
    ACCELERATION = "acceleration"
    FAULT_TYPE = "fault_type"
    FAULT_VALUE = "fault_value"
    JOINTS = "joints"
    DURATION = "duration"


class FaultTypes:
    """Types of faults that can be injected into the robot arm."""

    STUCK_JOINT = "stuck_joint"
    WEAR = "wear"


class RobotArmStateKeys:
    """Keys used in state messages sent from the robot arm."""

    ROBOT_MODE = "robot_mode"
    Q_ACTUAL = "q_actual"
    QD_ACTUAL = "qd_actual"
    Q_TARGET = "q_target"
    TIMESTAMP = "timestamp"
    JOINT_MAX_SPEED = "joint_max_speed"
    JOINT_MAX_ACCELERATION = "joint_max_acceleration"
    TCP_POSE = "tcp_pose"

class ParticleFilterMsgKeys:
    Q_ACTUAL = "q_actual"
    QD_ACTUAL = "qd_actual"
    TIMESTAMP = "timestamp"

class RobotMode:
    """Possible modes of the robot arm (ROBOT_MODE)."""

    ROBOT_MODE_RUNNING = "Running"
    ROBOT_MODE_IDLE = "Idle"


def encode_json(object):
    return json.dumps(object).encode(ENCODING)


def decode_json(bytes):
    return json.loads(bytes.decode(ENCODING))

def unroll_list(key_prefix, values):
    return {
        f"{key_prefix}_{i}": v
        for i, v in enumerate(values)
    }
