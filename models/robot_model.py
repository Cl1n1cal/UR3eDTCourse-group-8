from signal import pause

import numpy as np
import roboticstoolbox as rtb
import utils.calculation_functions as calc
from communication.protocol import RobotMode
import spatialmath as spm
import matplotlib.pyplot as plt

class RobotModel:
    def __init__(self, step_size: float = 0.01, d = [0.0], a = [0.0], alpha = [0.0]):
        self.q_current = np.zeros(6) 
        self.qd_current = np.zeros(6)
        self.qdd_current = np.zeros(6)
        self.q_end = np.zeros(6)
        self.tcp_pose = spm.SE3() # Position [0 0 0]
        self.trajectory = None

        self.max_velocity = 0.0
        self.max_acceleration = 0.0

        #deprecated, only used for stop trajectory
        self.phy_max_acceleration = 0.0
        self.phy_max_velocity = 0.0

        self.state = RobotMode.ROBOT_MODE_IDLE
        self.step_size = step_size
        self.current_traj_index = 0
        self.robot: rtb.DHRobot = create_robot(d, a, alpha)
    
    def setup_initial_state(self, q_start: np.ndarray, max_joint_velocity: float, max_joint_acceleration: float) -> None:
        self.q_current = q_start
        self.max_velocity =  max_joint_velocity
        self.max_acceleration = max_joint_acceleration
        self.phy_max_velocity = max_joint_velocity
        self.phy_max_acceleration = max_joint_acceleration
        self.tcp_pose = self.robot.fkine(self.q_current)

    def get_q_current(self) -> np.ndarray:
        return self.q_current
    
    def get_qd_current(self) -> np.ndarray:
        return self.qd_current / self.step_size
    
    def get_qdd_current(self) -> np.ndarray:
        return self.qdd_current / self.step_size**2
    
    def get_q_end(self) -> np.ndarray:
        return self.q_end
    
    def get_tcp_pose_current(self) -> spm.SE3:
        return self.tcp_pose

    def step(self):
        match self.state:
            case RobotMode.ROBOT_MODE_IDLE:
                return
            case RobotMode.ROBOT_MODE_RUNNING:
                self.iterate_trajectory()
                
    def iterate_trajectory(self):
        if self.trajectory is None:
            print("No trajectory to follow.")
            return
        
        traj_q = getattr(self.trajectory, "q", None)
        traj_qd = getattr(self.trajectory, "qd", None)
        traj_qdd = getattr(self.trajectory, "qdd", None)

        if traj_q is None or traj_qd is None or traj_qdd is None:
            print("Trajectory does not have expected fields.")
            return

        if self.current_traj_index < len(self.trajectory.q):
            self.q_current = traj_q[self.current_traj_index]
            self.qd_current = traj_qd[self.current_traj_index]
            self.qdd_current = traj_qdd[self.current_traj_index]
            self.tcp_pose = self.robot.fkine(self.q_current)
            self.current_traj_index += 1
        else:
            print("Reached end of trajectory.")
            self.stop()

    def load_program(self, q_end: np.ndarray, max_velocity: float, acceleration: float):
        self.q_end = q_end
        self.max_velocity = max_velocity
        self.max_acceleration = acceleration
        self.set_move_traj()

    def play(self):
        self.state = RobotMode.ROBOT_MODE_RUNNING

    def pause(self):
        self.state = RobotMode.ROBOT_MODE_IDLE

    def stop(self):
        self.state = RobotMode.ROBOT_MODE_IDLE
        self.clear_program()

    def clear_program(self):
        self.q_end = np.zeros(6)
        self.max_velocity = 0.0
        self.max_acceleration = 0.0
        self.trajectory = None

    #deprecated
    def set_halt_traj(self):
        self.current_traj_index = 0
        q_end = calc.compute_stop_q_end(self.q_current, self.qd_current, self.phy_max_acceleration)
        self.trajectory = rtb.jtraj(self.q_current, q_end, calc.compute_steps(self.q_current, q_end, self.phy_max_velocity, self.phy_max_acceleration, self.step_size), qd0=self.qd_current)
    
    def set_move_traj(self):
        self.current_traj_index = 0
        self.trajectory = rtb.mtraj(rtb.trapezoidal, self.q_current, self.q_end, calc.compute_steps(self.q_current, self.q_end, self.max_velocity, self.max_acceleration, self.step_size)) 

    def update_dh_parameters(self, d: list, a: list, alpha: list):
        print("Updating params")
        for i in range(6):
            self.robot.links[i] = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i])
        self.tcp_pose = self.robot.fkine(self.q_current)

@staticmethod
def create_robot(d: list, a: list, alpha: list):
    links = []
    for j in range(6):
        link = rtb.RevoluteDH(d=d[j], a=a[j], alpha=alpha[j])
        links.append(link)
    
    # 6 link robot
    return rtb.DHRobot([links[0], links[1], links[2], links[3], links[4], links[5]])
    