import numpy as np
import random
import math
import json
import roboticstoolbox as rtb
from models.robot_model import RobotModel
from communication.protocol import unroll_list
from communication.factory import RabbitMQFactory, ROUTING_KEY_PARTICLE, ROUTING_KEY_CTRL, RobotArmStateKeys, CtrlMsgFields, CtrlMsgKeys, ROUTING_KEY_CALIBRATION

# Factorial growth
sample_count = 300

rng = np.random.default_rng()

def generate_movement(n_moving, sample_count, from_list, to_list):
    for _ in range(sample_count):
        q_current = [random.uniform(-2*np.pi, 2*np.pi) for _ in range(6)]
        q_target = q_current.copy()
        moving_joints = random.sample(range(6), n_moving)
        for j in moving_joints:
            q_target[j] = random.uniform(-2*np.pi, 2*np.pi)
        from_list.append(q_current)
        to_list.append(q_target)


from_pos = []
target_pos = []

for n in range(1, 7):
    generate_movement(n, sample_count, from_pos, target_pos)

assert len(from_pos) == len(target_pos), "From pos and target pos are not the same length"

#index = 17501
#print(f"From: {from_pos[index]}\nTarget: {target_pos[index]}")

# Inputs: Current joint pos, current joint vels, target joint vels
# Outputs: Next joint vels, next joint pos

# Create random vel and acc values
"""
velocities = []
accelerations = []
for i in range(velocity_count):
    # Rounded to integer, ensure minimum of 1
    tmp = max(3, round(random.uniform(10, 60)))
    if tmp > 0.0:
        #tmp = np.deg2rad(tmp)
        velocities.append(tmp)

    # make sure that acc is smaller than vel and at least 1
    acc_val = max(1, round(random.uniform(1, tmp - 1)))
    if acc_val > 0.0:
        #acc_val = np.deg2rad(acc_val)
        accelerations.append(acc_val)

assert len(velocities) == len(accelerations), "Velocities and accelerations are not the same length"

for vel in velocities:
    if vel <= 0.0:
        raise Exception("Vel was < 0.0")
    
for acc in accelerations:
    if acc < 0.0:
        raise Exception("Acc was < 0.0")
"""
velocities_deg = [10, 20, 40, 70, 100, 130, 160, 180]   # deg/s
accelerations_deg = [5, 10, 20, 40, 70, 100, 130, 160]  # deg/s²

# Only keep combos where acc < vel (ensures interesting trajectory shapes)
combos = [(v, a) for v in velocities_deg for a in accelerations_deg if a < v]



# Create robot model
from utils.configuration import load_config; config = load_config('startup/startup.conf')
dh_params=config['digital_twin']['robot_model']['dh_parameters']
time_step=config['digital_twin']['robot_model']['time_step']

robot_model = RobotModel(step_size=time_step, d=dh_params['d'], a=dh_params['a'], alpha=dh_params['alpha'])

data = {}

# Generate a trajectory for all of the velocities and acceleration and for all the different positions
traj_counter = 0
file_counter = 29
for h in range(len(combos)):
    print(f"h:{h} of {len(combos)}")
    for k in range(len(from_pos)):
        # Skip if no movement is required (from_pos == target_pos)
        if np.allclose(from_pos[k], target_pos[k]):
            print(f"Skipping trajectory {k}: from_pos and target_pos are identical")
            continue
            
        # Set start pos
        robot_model.q_current = from_pos[k]

        # Set end pos and do calculation
        try:
            robot_model.load_program(target_pos[k], combos[h][0], combos[h][1])
            trajectory = robot_model.trajectory
        except RuntimeWarning and Exception:
            print("Skipping on exception")
            continue
        
        traj_q = getattr(trajectory, "q", None)
        traj_qd = getattr(trajectory, "qd", None)

        if traj_qd is None or traj_q is None:
            raise Exception("Traj_q or traj_qd was none")

        traj_q = traj_q.tolist()
        traj_qd = traj_qd.tolist()
        target_q = target_pos[k]
        max_vel = combos[h][0]
        max_acc = combos[h][1]

        """
        for i in range(len(traj_qd)):
            vels = traj_qd[i]
            for j in range(len(vels)):
                vels[j] = round(vels[j], 4)

        """

        trajectory = {}
        steps = {}
        joints = {}

        for i in range(len(traj_q)):
            vels = traj_qd[i]
            positions = traj_q[i]
            for j in range(len(positions)):
                joint = positions[j]
                joints[f"q_current_{j}"] = joint
            
            for j in range(len(vels)):
                vel = vels[j]
                joints[f"qd_current_{j}"] = vel

            for j in range(len(target_q)):
                joint = target_q[j]
                joints[f"q_target_{j}"] = joint
            
            # Add vel, acc and timestep
            joints["max_vel"] = np.deg2rad(max_vel)
            joints["max_acc"] = np.deg2rad(max_acc)
            joints["time_step"] = 0.025

            steps[f"step_{i}"] = joints
            joints = {}




        #trajectory["traj_q"] = traj_q.tolist()
        #trajectory["traj_qd"] = traj_qd.tolist()

        data[f"Trajectory_{traj_counter}"] = steps
        steps = {}

        traj_counter += 1

        # Write 1000 trajectories to every file to prevent getting a huge file
        if traj_counter == 1000:
            with open(f"nn_folder/nn_training_data/trajectories_{file_counter}.json", "w") as file:
                json.dump(data, file, indent=4)

            # Reset traj counter, increment file counter, reset data
            traj_counter = 0
            file_counter += 1
            data = {}

# Write the last one even if it is not 1000 trajs
if data:
    with open(f"nn_folder/nn_training_data/trajectories_{file_counter}.json", "w") as file:
        json.dump(data, file, indent=4)

