import pandas as pd
import json
import torch
import torch.nn as nn
import torch.optim as optim

# Creating the model
# Define the neural network model (a simple linear regression model)
class LinearRegression(nn.Module):
    def __init__(self):
        super(LinearRegression, self).__init__()
        self.linear = nn.Linear(18, 12)  # 24 input features and 12 output

    def forward(self, x):
        return self.linear(x)

# Initialize the model and define the loss function (Mean Squared Error)
model = LinearRegression()

learning_rate = 0.01
optimizer = optim.Adam(model.parameters(), lr=learning_rate)
num_epochs = 20000  # Number of epochs to train

# Define the loss function (Mean Squared Error)
loss_fn = nn.MSELoss()


data = None

for q in range(15):
    print("Q:", q)
    with open(f"nn_data/trajectories_{q}.json", "r") as file:
        data = json.load(file)

    # List of data frames
    dfs = []
    x = []
    y = []
    for d in data.keys():
        trajectory = data[d]
        for steps in trajectory.keys():
            single_step = trajectory[steps]
            tmp_x = []
            tmp_y = []

            for i in range(6):
                tmp_x.append(single_step[f"q_current_{i}"])
                tmp_y.append(single_step[f"q_current_{i}"])
            
            for i in range(6):
                tmp_x.append(single_step[f"qd_current_{i}"])
                tmp_y.append(single_step[f"qd_current_{i}"])

            for i in range(6):
                tmp_x.append(single_step[f"q_target_{i}"])
            
            x.append(tmp_x)
            y.append(tmp_y)


    # Make sure that the data of the lists match
    # This is because we will use the x values to get the next values which should be equal to y
    x = x[0:-1] # Get all but the last element of x
    y = y[1::] # Get all but the first element of y

    assert len(x) == len(y), "X and y are not the same length"

    X_train = torch.tensor(x)
    Y_train = torch.tensor(y)
    #print("len x:", len(x))
    #print("len y:", len(y))
    #print("X_train", X_train)
    #print("Y_train", Y_train)


    # Training loop
    for epoch in range(num_epochs):
        optimizer.zero_grad()  # Zero out gradients
        outputs = model(X_train)  # Forward pass
        loss = loss_fn(outputs, Y_train)  # Compute the loss
        loss.backward()  # Backpropagation
        
        optimizer.step()  # Update model parameters

        # Print loss periodically for monitoring
        if (epoch + 1) % 500 == 0:
            print(f'Epoch [{epoch + 1}/{num_epochs}], Loss: {loss.item():.4f}')

torch.save(model.state_dict(), 'nn_model.pth')