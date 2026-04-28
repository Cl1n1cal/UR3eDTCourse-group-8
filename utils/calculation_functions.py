import numpy as np
from spatialmath import SE3

def compute_time(q_start: np.ndarray, q_end: np.ndarray, v_max: float, a_max: float, dt: float):
    """
    Positions: radians
    Velocity: rad/s
    Acceleration: rad/s^2
    """

    T_all = []

    for i in range(len(q_start)):
        delta_q = abs(q_end[i] - q_start[i])

        t_acc = v_max / a_max
        q_acc = dt * a_max * t_acc**2
        q_acc_total = 2 * q_acc

        if delta_q > q_acc_total:
            # Trapezoidal profile
            q_const = delta_q - q_acc_total
            t_const = q_const / v_max
            T_i = 2 * t_acc + t_const
        else:
            # Triangular profile
            T_i = 2 * np.sqrt(delta_q / a_max)

        T_all.append(T_i)

    T_total = max(T_all)

    return T_total

def compute_steps(q_start: np.ndarray, q_end: np.ndarray, v_max: float, a_max: float, dt: float):
    """
    Positions: radians
    Velocity: rad/s
    Acceleration: rad/s^2
    """

    T_all = []

    for i in range(len(q_start)):
        delta_q = abs(q_end[i] - q_start[i])

        t_acc = v_max / a_max
        q_acc = 0.5 * a_max * t_acc**2
        q_acc_total = 2 * q_acc

        if delta_q > q_acc_total:
            # Trapezoidal profile
            q_const = delta_q - q_acc_total
            t_const = q_const / v_max
            T_i = 2 * t_acc + t_const
        else:
            # Triangular profile
            T_i = 2 * np.sqrt(delta_q / a_max)

        T_all.append(T_i)

    T_total = max(T_all)
    n_steps = int(np.ceil(T_total / dt))

    return n_steps

def compute_steps_per_joint(q_start: np.ndarray, q_end: np.ndarray, v_max: float, a_max: float, dt: float):
    """
    Positions: radians
    Velocity: rad/s
    Acceleration: rad/s^2
    """

    # 1. Find the joint with the max distance
    delta_qs = [abs(q_end[i] - q_start[i]) for i in range(len(q_start))]
    idx_max = int(np.argmax(delta_qs))
    delta_q_max = delta_qs[idx_max]

    # 2. Compute the time and steps for the max joint
    t_acc = v_max / a_max
    q_acc = 0.5 * a_max * t_acc**2
    q_acc_total = 2 * q_acc
    if delta_q_max > q_acc_total:
        # Trapezoidal profile
        q_const = delta_q_max - q_acc_total
        t_const = q_const / v_max
        T_total = 2 * t_acc + t_const
        vmax_max = v_max
    else:
        # Triangular profile
        T_total = 2 * np.sqrt(delta_q_max / a_max)
        vmax_max = a_max * (T_total / 2)
    steps = int(np.ceil(T_total / dt))

    # 3. For all joints, scale the peak velocity by the proportion of distance
    vmax_all = []
    for i in range(len(q_start)):
        if delta_q_max == 0:
            vmax_i = 0.0
        else:
            vmax_i = vmax_max * (delta_qs[i] / delta_q_max)
        vmax_all.append(vmax_i)

    return steps, vmax_all

def compute_T_jtraj(delta_q, v_max, a_max):
    """
    delta_q : displacement (radians or degrees)
    v_max   : max allowed velocity
    a_max   : max allowed acceleration
    """
    T_v = 1.875 * delta_q / v_max
    T_a = np.sqrt(5.77 * delta_q / a_max)

    # Take the larger one to satisfy both constraints
    T = max(T_v, T_a)
    return T

def compute_steps_jtraj(q_start, q_end, v_max, a_max, dt):
    T_all = []
    for i in range(len(q_start)):
        delta_q = abs(q_end[i] - q_start[i])
        T_i = compute_T_jtraj(delta_q, v_max, a_max)
        T_all.append(T_i)

    T_total = max(T_all)  # synchronous joint motion
    n_steps = int(np.ceil(T_total / dt))
    return n_steps


def compute_stop_q_end(q_start, v_current, a_max):
    stop_dist = 0.5 * (v_current ** 2) / a_max
    q_end = q_start + stop_dist * np.sign(v_current)

    return q_end

def se3_to_pos_rpy(se3: SE3):
    tcp_rpy = se3.rpy(order='xyz')
    tcp_xyz = se3.t
    return np.hstack((tcp_xyz, tcp_rpy))

def milliseconds_to_seconds(milliseconds: float) -> float:
    return milliseconds / 1000.0