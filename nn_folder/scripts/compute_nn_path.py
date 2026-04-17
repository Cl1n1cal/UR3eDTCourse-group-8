from nn_folder.classes.linear_regression_single import LinearRegressionSingle
from nn_folder.classes.robot_prediction_nn import RobotPredictionNN
from utils.configuration import load_config
import numpy as np
from models.robot_model import RobotModel
import matplotlib.pyplot as plt


nn_model = RobotPredictionNN()
config = load_config("startup/startup.conf")
config = config["particle_filter_service"]
nn_model.setup(config["nn_model_path"])

# Create robot model
from utils.configuration import load_config; config = load_config('startup/startup.conf')
dh_params=config['digital_twin']['robot_model']['dh_parameters']
time_step=config['digital_twin']['robot_model']['time_step']

robot_model = RobotModel(step_size=time_step, d=dh_params['d'], a=dh_params['a'], alpha=dh_params['alpha'])

# Robot model requires nd array
q_start = np.array([0, 0, np.pi/2, 0, -np.pi/2, 0])
q_end = np.array([0, 0, np.pi/2, 2*np.pi, -np.pi/2, 0])

# NN requires lists
#q_start_list = q_start.tolist()
#q_end_list = q_end.tolist()
q_vels = [0,0,0,0,0,0]

max_vel = 80
max_acc = 30

robot_model.q_current = q_start
robot_model.load_program(q_end, max_vel, max_acc)
trajectory = robot_model.trajectory

traj_q = getattr(trajectory, "q", None)
traj_qd = getattr(trajectory, "qd", None)

if traj_qd is None or traj_q is None:
    raise Exception("Traj_q or traj_qd was none")

traj_q = traj_q.tolist()
traj_qd = traj_qd.tolist()

steps = len(traj_q)

constraints = [max_vel, max_acc, 0.025]

init_vals = np.concatenate([q_start, q_vels, q_end, constraints])
init_vals = init_vals.astype(np.float32)

vals = []
results = []
sim = []
for i in range(steps):
    sim.append(traj_q[i][3])
 
    if i == 0:
        nn_model.predict(init_vals)
        vals = nn_model.get_prediction().tolist()
        results.append(vals)
    else:
        [vals.append(x) for x in q_end]
        vals.append(max_vel)
        vals.append(max_acc)
        vals.append(0.025)
        vals = np.array(vals)
        vals = vals.astype(np.float32)
        nn_model.predict(vals)
        vals = nn_model.get_prediction().tolist()
        results.append(vals)

model = []
nn = []

print("len res:", len(results))
print("steps:", steps)

for i in range(len(results)):
    print("i:", i)
    nn.append(results[i][3])

print("len nn", len(nn))
print("nn", nn)
x_axis = list(range(0, len(nn)))
plt.plot(x_axis, nn, label='NN', color='magenta')
plt.plot(x_axis, sim, label='SIM', color='red')
plt.legend()
plt.show()