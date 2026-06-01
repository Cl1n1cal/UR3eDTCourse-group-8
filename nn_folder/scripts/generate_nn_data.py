import numpy as np
import random
import json
from models.robot_model import RobotModel

# Grows with combo of velocities and accelerations as well
sample_count = 300

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

# more samples for single joint
generate_movement(1, 800, from_pos, target_pos)  
generate_movement(2, 300, from_pos, target_pos)
generate_movement(3, 300, from_pos, target_pos)
generate_movement(4, 300, from_pos, target_pos)
generate_movement(5, 300, from_pos, target_pos)
generate_movement(6, 300, from_pos, target_pos)


assert len(from_pos) == len(target_pos), "From pos and target pos are not the same length"

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
file_counter = 0
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

