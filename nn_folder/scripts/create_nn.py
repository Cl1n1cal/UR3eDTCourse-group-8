import pandas as pd
import json
import torch
import torch.nn as nn
import torch.optim as optim
from nn_folder.classes.linear_regression_multi import LinearRegressionMulti
from nn_folder.classes.linear_regression_single import LinearRegressionSingle
from torch.utils.data import TensorDataset, DataLoader




# Initialize the model and define the loss function (Mean Squared Error)
model = LinearRegressionMulti()
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("cuda available:", torch.cuda.is_available())
model = model.to(device) 
learning_rate = 0.001
optimizer = optim.Adam(model.parameters(), lr=learning_rate)

# Define the loss function (Mean Squared Error)
loss_fn = nn.MSELoss()


data = None


traj_count = 94
input_steps = []
output_steps = []
for q in range(traj_count):
    print(f"Loading traj: {q} of {traj_count}")
    with open(f"nn_folder/nn_training_data/trajectories_{q}.json", "r") as file:
        data = json.load(file)

    # List of data frames
    dfs = []

    for d in data.keys():
        trajectory = data[d]
        traj_x = []
        traj_y = []

        for steps in trajectory.keys():
            single_step = trajectory[steps]
            step_x = []
            step_y = []

            for i in range(6):
                step_x.append(single_step[f"q_current_{i}"])
                step_y.append(single_step[f"q_current_{i}"])
            
            for i in range(6):
                step_x.append(single_step[f"qd_current_{i}"])
                step_y.append(single_step[f"qd_current_{i}"])

            for i in range(6):
                step_x.append(single_step[f"q_target_{i}"])
            
            step_x.append(single_step["max_vel"])
            step_x.append(single_step["max_acc"])
            step_x.append(single_step["time_step"])
    
            traj_x.append(step_x)
            traj_y.append(step_y)
        
        for i in range(len(traj_x) - 1):
            input_steps.append(traj_x[i])
            output_steps.append(traj_y[i+1])



all_inputs = input_steps
all_outputs = output_steps

assert len(all_inputs) == len(all_outputs), "Inputs and outputs are not the same length"

X_all = torch.tensor(all_inputs).float()
Y_all = torch.tensor(all_outputs).float()
X_all = X_all.to(device)
Y_all = Y_all.to(device)

dataset = TensorDataset(X_all, Y_all)
loader = DataLoader(dataset, batch_size=512, shuffle=True)

num_epochs = 200  # Number of epochs to train
for epoch in range(num_epochs):
    epoch_loss = 0
    for X_batch, Y_batch in loader:
        optimizer.zero_grad()
        loss = loss_fn(model(X_batch), Y_batch)
        loss.backward()
        optimizer.step()
        epoch_loss += loss.item()
    print(f"Epoch {epoch+1}/{num_epochs}, Loss: {epoch_loss/len(loader):.6f}")


torch.save(model.state_dict(), 'nn_folder/models/nn_model_multi_layer_new.pth')