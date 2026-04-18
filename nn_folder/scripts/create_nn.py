import pandas as pd
import json
import torch
import torch.nn as nn
import torch.optim as optim
from nn_folder.classes.linear_regression_multiple import LinearRegressionMulti
from nn_folder.classes.linear_regression_single import LinearRegressionSingle


# Initialize the model and define the loss function (Mean Squared Error)
model = LinearRegressionMulti()
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("cuda available:", torch.cuda.is_available())
model = model.to(device) 
learning_rate = 0.01
optimizer = optim.Adam(model.parameters(), lr=learning_rate)
num_epochs = 200  # Number of epochs to train

# Define the loss function (Mean Squared Error)
loss_fn = nn.MSELoss()


data = None


traj_count = 40
for q in range(traj_count):
    traj_list_input = []
    traj_list_output = []

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
        
        traj_list_input.append(traj_x)
        traj_list_output.append(traj_y)


    assert len(traj_list_input) == len(traj_list_output), "Traj_list_input and traj_list_output are not the same length"

    for i in range(len(traj_list_input)):
        traj_list_input[i] = traj_list_input[i][0:-1]  # Get all but the last element of every trajectory
        traj_list_output[i] = traj_list_output[i][1::] # Get all but the first element of every trajectory
        assert len(traj_list_input[i]) == len(traj_list_output[i]), "Traj input and traj output are not the same length"


#print("len x:", len(x))
#print("len y:", len(y))
#print("X_train", X_train)
#print("Y_train", Y_train)

# TODO: Convert to correct float and also recreate training data with rads instead of degs

# Training loop

    for i in range(len(traj_list_input)):
        if i % 100 == 0:
            print(f"pct. trough traj {q} of {traj_count}: {round((i/len(traj_list_input))*100, 2)}")

        X_train = torch.tensor(traj_list_input[i]).float()
        Y_train = torch.tensor(traj_list_output[i]).float()
        X_train = X_train.to(device)
        Y_train = Y_train.to(device)

        for epoch in range(num_epochs):
            optimizer.zero_grad()  # Zero out gradients
            outputs = model(X_train)  # Forward pass
            loss = loss_fn(outputs, Y_train)  # Compute the loss
            loss.backward()  # Backpropagation
            
            optimizer.step()  # Update model parameters

            # Print loss periodically for monitoring
            #if (epoch + 1) % 100 == 0:
            #    print(f'Epoch [{epoch + 1}/{num_epochs}], Loss: {loss.item():.4f}')

torch.save(model.state_dict(), 'nn_folder/models/nn_model_multi_layer_new.pth')