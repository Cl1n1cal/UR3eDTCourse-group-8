import numpy as np
import json
import matplotlib.pyplot as plt
from nn_folder.classes.robot_prediction_nn import RobotPredictionNN
import torch
import torch.nn as nn

mockup_data = []
sim_data = []

nn_model = RobotPredictionNN()
nn_model.setup("nn_folder/models/nn_model_single_layer.pth")

with open("mockup_results_rounded.json", "r") as file:
    mockup_data = json.load(file)

with open("sim_results_rounded.json", "r") as file:
    sim_data = json.load(file)


mockup_steps = [] # List of list: List of joints positions(list)
sim_steps = []
nn_input_list = []

for elem in sim_data:
    tmp = []
    tmp.append(elem.get("q_actual_0"))
    tmp.append(elem.get("q_actual_1"))
    tmp.append(elem.get("q_actual_2"))
    tmp.append(elem.get("q_actual_3"))
    tmp.append(elem.get("q_actual_4"))
    tmp.append(elem.get("q_actual_5"))
    tmp.append(elem.get("qd_actual_0"))
    tmp.append(elem.get("qd_actual_1"))
    tmp.append(elem.get("qd_actual_2"))
    tmp.append(elem.get("qd_actual_3"))
    tmp.append(elem.get("qd_actual_4"))
    tmp.append(elem.get("qd_actual_5"))
    tmp.append(0)
    tmp.append(0)
    tmp.append(np.pi/2)
    tmp.append(0)
    tmp.append(-np.pi/2)
    tmp.append(0)

    nn_input_list.append(tmp)

# Get joints values from the mockup
for elem in mockup_data:
    tmp = []
    tmp.append(elem.get("q_actual_0"))
    tmp.append(elem.get("q_actual_1"))
    tmp.append(elem.get("q_actual_2"))
    tmp.append(elem.get("q_actual_3"))
    tmp.append(elem.get("q_actual_4"))
    tmp.append(elem.get("q_actual_5"))
    tmp.append(elem.get("qd_actual_0"))
    tmp.append(elem.get("qd_actual_1"))
    tmp.append(elem.get("qd_actual_2"))
    tmp.append(elem.get("qd_actual_3"))
    tmp.append(elem.get("qd_actual_4"))
    tmp.append(elem.get("qd_actual_5"))

    mockup_steps.append(tmp)

for elem in sim_data:
    tmp = []
    tmp.append(elem.get("q_actual_0"))
    tmp.append(elem.get("q_actual_1"))
    tmp.append(elem.get("q_actual_2"))
    tmp.append(elem.get("q_actual_3"))
    tmp.append(elem.get("q_actual_4"))
    tmp.append(elem.get("q_actual_5"))
    tmp.append(elem.get("qd_actual_0"))
    tmp.append(elem.get("qd_actual_1"))
    tmp.append(elem.get("qd_actual_2"))
    tmp.append(elem.get("qd_actual_3"))
    tmp.append(elem.get("qd_actual_4"))
    tmp.append(elem.get("qd_actual_5"))

    sim_steps.append(tmp)


sim_steps = sim_steps[0:-1] # Get all but the last element of x
mockup_steps = mockup_steps[1::] # Get all but the first element of y

assert len(mockup_steps) == len(sim_steps), "Mockup joints and sim joints are not the same length"

# Measurement noise parameters (GPS-like noise)
# TODO: Find out if speed has the same insecurity
mockup_noise = 0.04 #0.00003  # Standard deviation (m) taken from UR3e datasheet: https://www.universal-robots.com/media/1807464/ur3e_e-series_datasheets_web.pdf

# Particle filter
#  parameters
N = 100000
#particles = np.random.normal(0, 2*np.pi, num_particles)  # Initialize particles around 0
particles = []
weights = []

def wrap_to_pi(x):
    return (x + np.pi) % (2 * np.pi) - np.pi
# Create 12 particle and weight lists
mean = np.array(sim_steps[0])  # first timestep
std  = np.array([0.1]*6 + [0.5]*6)

particles = mean + std * np.random.randn(N, 12)
particles[:, :6] = wrap_to_pi(particles[:, :6])
# Lists to store results
true_positions = []
sim_positions = []
mockup_measurements = []
pf_estimates = []  # Particle filter estimated positions


q_target = np.array([0, 0, np.pi/2, 0, -np.pi/2, 0])

# Repeat for all particles
#q_target_batch = q_target.unsqueeze(0).repeat(N, 1)  # (5000, 6)
q_target_batch = np.tile(q_target, (N, 1))


#joints = particles[:, 0:6]
#velocities = particles[:, 6:12]

#nn_input = np.concatenate([joints, velocities, q_target_batch], axis=1)

# Particle Filter Process
# For every step in total steps


for i in range(len(sim_steps)):

    # --------------------------
    # 1. VECTORIZE STATE SPLIT
    # --------------------------
    joints = particles[:, 0:6]
    velocities = particles[:, 6:12]

    # --------------------------
    # 2. BUILD NN INPUT (BATCH)
    # --------------------------
    nn_input = np.concatenate([joints, velocities, q_target_batch], axis=1)
    nn_input = nn_input.astype(np.float32)

    # --------------------------
    # 3. NN PREDICTION (BATCH)
    # --------------------------
    nn_model.predict(nn_input)
    pred = np.array(nn_model.get_prediction())

    # --------------------------
    # 4. ADD NOISE
    # --------------------------
    noise = np.random.normal(0, 0.22, (N, 12))
    particles = pred + noise

    # --------------------------
    # 5. WRAP ANGLES
    # --------------------------
    particles[:, :6] = wrap_to_pi(particles[:, :6])

    # --------------------------
    # 6. MEASUREMENT UPDATE
    # --------------------------
    measurement = np.array(mockup_steps[i])

    error = particles - measurement

    weights = np.exp(-0.5 * np.sum((error / mockup_noise)**2, axis=1))
    weights += 1e-300
    weights /= np.sum(weights)

    # --------------------------
    # 7. RESAMPLING
    # --------------------------
    indices = np.random.choice(N, N, p=weights)
    particles = particles[indices]

    # --------------------------
    # 8. ESTIMATE
    # --------------------------
    estimate = np.mean(particles, axis=0)

    pf_estimates.append(estimate[3])
    sim_positions.append(sim_steps[i][3])
    mockup_measurements.append(mockup_steps[i][3])


# Plot results
plt.figure(figsize=(10, 5))
#plt.plot(true_positions, label="True Position (Real World)", linestyle='dashed')
plt.plot(sim_positions, label="Simulated Position (Model w/ Drift)", linestyle='solid')
plt.scatter(range(len(mockup_measurements)), mockup_measurements, label="Mockup Measurements (Noisy)", color='red', s=5)
#plt.plot(sim_q3, label="Simulated Position (Model w/ Drift)", linestyle='solid')
#plt.scatter(range(len(mockup_measurements)), mockup_q3, label="Mockup Measurements (Noisy)", color='red', s=5)
plt.plot(pf_estimates, label="Particle Filter Estimate", linestyle='dotted', color='green')
plt.xlabel("Time Step")
plt.ylabel("Position (m)")
plt.legend()
plt.title("Particle Filter Correcting Simulation Drift")
plt.show()