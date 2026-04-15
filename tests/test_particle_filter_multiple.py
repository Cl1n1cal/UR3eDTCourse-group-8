import numpy as np
import json
import matplotlib.pyplot as plt
from nn_folder.classes.robot_prediction_nn import RobotPredictionNN

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
mockup_noise = 0.00003  # Standard deviation (m) taken from UR3e datasheet: https://www.universal-robots.com/media/1807464/ur3e_e-series_datasheets_web.pdf

# Particle filter parameters
num_particles = 5000
#particles = np.random.normal(0, 2*np.pi, num_particles)  # Initialize particles around 0
particles = []
weights = []

# Create 12 particle and weight lists
for i in range(12):
    particles.append(np.random.normal(0, 2, num_particles))  # Initialize particles around 0
    weights.append(np.ones(num_particles) / num_particles)  # Uniform weights

# Lists to store results
true_positions = []
sim_positions = []
mockup_measurements = []
pf_estimates = []  # Particle filter estimated positions

# Particle Filter Process
# For every step in total steps
for i in range(len(mockup_steps)):

    sim_step = sim_steps[i]
    mockup_step = mockup_steps[i]
    nn_input = nn_input_list[i]
    nn_model.predict(nn_input)
    nn_prediction_list = nn_model.get_prediction().tolist()

    # For every joint and velocity
    for j in range(12):
        # --- Simulated Motion (Model with Drift) ---
        
        sim_joint = sim_step[j]

        # --- Mockup Measurement (with Noise) ---
        mockup_joint = mockup_step[j]

        nn_prediction = nn_prediction_list[j]

        # --- Particle Filter Update ---
        # 1. Motion Update: Particles drift slightly from simulation prediction
        process_noise = 0.02  # How much particles can deviate from sim
        particles[j] = nn_prediction + np.random.normal(0, 3, num_particles)  # Initialize particles around 0

        # 2. Measurement Update: Weight particles based on mockup (noisy measurement)
        weights[j] = np.exp(-0.5 * ((particles[j] - mockup_joint) / mockup_noise) ** 2)
        weights[j] += 1e-300  # Avoid zeros
        weights[j] /= np.sum(weights[j])  # Normalize weights

        # 3. Resampling: Draw new particles based on weights
        indices = np.random.choice(range(num_particles), size=num_particles, p=weights[j])
        particles[j] = particles[j][indices]

        # Store estimated position as the mean of particles
        pf_estimate = np.mean(particles[j])

        # Store values
        #true_positions.append(true_x)
        if j == 3:
            sim_positions.append(sim_joint)
            mockup_measurements.append(mockup_joint)
            pf_estimates.append(pf_estimate)

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