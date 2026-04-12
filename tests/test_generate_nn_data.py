import numpy as np
import random
import math
import json
import roboticstoolbox as rtb
from models.robot_model import RobotModel
from communication.protocol import unroll_list
from communication.factory import RabbitMQFactory, ROUTING_KEY_MODEL_STATE, ROUTING_KEY_CTRL, RobotArmStateKeys, CtrlMsgFields, CtrlMsgKeys, ROUTING_KEY_CALIBRATION

# Factorial growth
sample_count = 100

# Scaled the factorial sample_count with the amount specified
velocity_count = 25

rng = np.random.default_rng()

def generate_from_and_target_single_joint(joint):
    if joint < 0 or 5 < joint:
        raise Exception("joint out of bounds")

    from_pos = []
    target_pos = []

    for i in range(sample_count):

        # Create from_pos
        for k in range(6):
            tmp = []
            if k == joint:
                r_num = random.uniform(-2*np.pi, 2*np.pi)
                tmp.append(r_num)
            else:
                tmp.append(0)
        
        from_pos.append(tmp)

        # Create target_pos
        for k in range(6):
            tmp = []
            if k == joint:
                r_num = random.uniform(-2*np.pi, 2*np.pi)
                tmp.append(r_num)
            else:
                tmp.append(0.0)
        
        target_pos.append(tmp)
    
    return from_pos, target_pos

def generate_from_pos_target_pos_range(start, end, from_pos, target_pos):
    if start < 0 or end < 0 or 5 < start or 5 < end or end < start:
        raise Exception("joint out of bounds")
    
    for i in range(sample_count):
        tmp = []

        # Create start_pos
        for k in range(6):
            if start <= k and k <= end:
                r_num = round(random.uniform(-2*np.pi, 2*np.pi), 4)
                tmp.append(r_num)
            else:
                tmp.append(0.0)
        
        from_pos.append(np.array(tmp))
        tmp1 = []
    
        # Create target_pos
        for k in range(6):
            if start <= k and k <= end:
                #r_num = round(random.uniform(-2*np.pi, 2*np.pi), 4)
                if tmp[k] < 0.0:
                    tmp1.append(tmp[k] + 2)
                else:
                    tmp1.append(tmp[k] - 2)
            else:
                tmp1.append(0.0)
        
        target_pos.append(np.array(tmp1))


def generate_training_set(from_pos, target_pos):
    for i in range(6):
        for j in range(i, 6):
            generate_from_pos_target_pos_range(i, j, from_pos=from_pos, target_pos=target_pos)


from_pos = []
target_pos = []

generate_training_set(from_pos=from_pos, target_pos=target_pos)

assert len(from_pos) == len(target_pos), "From pos and target pos are not the same length"

#index = 17501
#print(f"From: {from_pos[index]}\nTarget: {target_pos[index]}")

# Inputs: Current joint pos, current joint vels, target joint vels
# Outputs: Next joint vels, next joint pos

# Create random vel and acc values
velocities = []
accelerations = []
for i in range(velocity_count):
    # Rounded to interger
    tmp = round(random.uniform(10,100))
    velocities.append(tmp)

    # make sure that acc is smaller than vel
    accelerations.append(round(random.uniform(10,tmp-1)))

assert len(velocities) == len(accelerations), "Velocities and accelerations are not the same length"

print("Vels:", velocities)
print("Accs:", accelerations)

# Create robot model
from utils.configuration import load_config; config = load_config('startup/startup.conf')
dh_params=config['digital_twin']['robot_model']['dh_parameters']
time_step=config['digital_twin']['robot_model']['time_step']

robot_model = RobotModel(step_size=time_step, d=dh_params['d'], a=dh_params['a'], alpha=dh_params['alpha'])

data = {}

# Generate a trajectory for all of the velocities and acceleration and for all the different positions
traj_counter = 0
file_counter = 0
for h in range(len(velocities)):
    print("Velocities:", h)

    for k in range(len(from_pos)):
        # Set start pos
        robot_model.setup_initial_state(from_pos[k], velocities[h], accelerations[h])
        # Set end pos and do calculation
        robot_model.load_program(target_pos[k], velocities[h], accelerations[h])

        trajectory = robot_model.trajectory

        traj_q = getattr(trajectory, "q", None)
        traj_qd = getattr(trajectory, "qd", None)

        trajectory = {}
        trajectory["traj_q"] = traj_q.tolist()
        trajectory["traj_qd"] = traj_qd.tolist()

        data[f"Trajectory_{traj_counter}"] = trajectory

        traj_counter += 1

        # Write 1000 trajectories to every file to prevent getting a huge file
        if traj_counter == 1000:
            with open(f"nn_data/trajectories_{file_counter}.json", "w") as file:
                json.dump(data, file, indent=4)

            # Reset traj counter, increment file counter, reset data
            traj_counter = 0
            file_counter += 1
            data = {}

