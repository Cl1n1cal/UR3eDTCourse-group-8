import numpy as np
import json
import matplotlib.pyplot as plt

mockup_data = []
sim_data = []

with open("mockup_results_rounded.json", "r") as file:
    mockup_data = json.load(file)

with open("sim_results_rounded.json", "r") as file:
    sim_data = json.load(file)

mockup_q3 = []
sim_q3 = []

for elem in mockup_data:
    mockup_q3.append(elem.get("q_actual_3"))

for elem in sim_data:
    sim_q3.append(elem.get("q_actual_3"))

assert len(mockup_q3) == len(sim_q3), "Mockup q3 and sim q3 are not the same length"

# Time step and duration
dt = 0.1  # Time step (s)
T = 20  # Total simulation time (s)
steps = int(T / dt)

# True car dynamics (real-world)
true_x = 0  # Initial position
true_v = 2  # True velocity (m/s)
true_a = 0.1  # True acceleration (m/s^2)

# Simulated car dynamics (imperfect model)
sim_x = 0
sim_v = 2
sim_a = 0.08  # Slightly incorrect acceleration

# Measurement noise parameters (GPS-like noise)
mockup_noise = 0.00003  # Standard deviation (m) taken from UR3e datasheet: https://www.universal-robots.com/media/1807464/ur3e_e-series_datasheets_web.pdf

# Particle filter parameters
num_particles = 1000
#particles = np.random.normal(0, 2*np.pi, num_particles)  # Initialize particles around 0
particles = np.random.normal(0, 1, num_particles)  # Initialize particles around 0
weights = np.ones(num_particles) / num_particles  # Uniform weights

# Lists to store results
true_positions = []
sim_positions = []
mockup_measurements = []
pf_estimates = []  # Particle filter estimated positions

# Particle Filter Process
for i in range(len(mockup_q3)):
    # --- Simulated Motion (Model with Drift) ---
    sim_val = sim_q3[i]

    # --- Mockup Measurement (with Noise) ---
    mockup_val = mockup_q3[i] # Measurement with noise

    # --- Particle Filter Update ---
    # 1. Motion Update: Particles drift slightly from simulation prediction
    process_noise = 0.02  # How much particles can deviate from sim
    particles = np.random.normal(sim_val, process_noise, num_particles)

    # 2. Measurement Update: Weight particles based on mockup (noisy measurement)
    weights = np.exp(-0.5 * ((particles - mockup_val) / mockup_noise) ** 2)
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