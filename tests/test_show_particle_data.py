import json
import matplotlib.pyplot as plt
import numpy as np

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

x_axis = []

for i in range(len(mockup_q3)):
    x_axis.append(i)


plt.plot(x_axis, mockup_q3, x_axis, sim_q3)
plt.show()