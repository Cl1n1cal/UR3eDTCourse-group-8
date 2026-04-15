import numpy as np
import json
import matplotlib.pyplot as plt

mockup_data = []
sim_data = []

with open("mockup_results_rounded.json", "r") as file:
    mockup_data = json.load(file)

with open("sim_results_rounded.json", "r") as file:
    sim_data = json.load(file)


mockup_steps = [] # List of list: List of joints positions(list)
sim_steps = []

# Get joints values from the mockup
for elem in mockup_data:
    tmp = []
    tmp.append(elem.get("q_actual_0"))
    tmp.append(elem.get("q_actual_1"))
    tmp.append(elem.get("q_actual_2"))
    tmp.append(elem.get("q_actual_3"))
    tmp.append(elem.get("q_actual_4"))
    tmp.append(elem.get("q_actual_5"))

    mockup_steps.append(tmp)

for elem in sim_data:
    tmp = []
    tmp.append(elem.get("q_actual_0"))
    tmp.append(elem.get("q_actual_1"))
    tmp.append(elem.get("q_actual_2"))
    tmp.append(elem.get("q_actual_3"))
    tmp.append(elem.get("q_actual_4"))
    tmp.append(elem.get("q_actual_5"))

    sim_steps.append(tmp)

assert len(mockup_steps) == len(sim_steps), "Mockup joints and sim joints are not the same length"

# Measurement noise parameters (GPS-like noise)
mockup_noise = 0.00003  # Standard deviation (m) taken from UR3e datasheet: https://www.universal-robots.com/media/1807464/ur3e_e-series_datasheets_web.pdf

# Particle filter parameters
num_particles = 1000
#particles = np.random.normal(0, 2*np.pi, num_particles)  # Initialize particles around 0
particles = []

# Create 6 particle lists
for i in range(6):
    particles.append(np.random.normal(0, 1, num_particles))  # Initialize particles around 0


weights = np.ones(num_particles) / num_particles  # Uniform weights

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

    # For every joint
    for j in range(6):
        # --- Simulated Motion (Model with Drift) ---
        
        sim_joint = sim_step[j]

        # --- Mockup Measurement (with Noise) ---
        mockup_joint = mockup_step[j]

        # --- Particle Filter Update ---
        # 1. Motion Update: Particles drift slightly from simulation prediction
        process_noise = 0.02  # How much particles can deviate from sim
        particles[j] = np.random.normal(0, 1, num_particles)  # Initialize particles around 0

        # 2. Measurement Update: Weight particles based on mockup (noisy measurement)
        weights = np.exp(-0.5 * ((particles[j] - mockup_joint) / mockup_noise) ** 2)
        weights += 1e-300  # Avoid zeros
        weights /= np.sum(weights)  # Normalize weights

        # 3. Resampling: Draw new particles based on weights
        indices = np.random.choice(range(num_particles), size=num_particles, p=weights)
        particles = particles[indices]

        # Store estimated position as the mean of particles
        pf_estimate = np.mean(particles)

        # Store values
        #true_positions.append(true_x)
        sim_positions.append(sim_val)
        mockup_measurements.append(mockup_val)
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