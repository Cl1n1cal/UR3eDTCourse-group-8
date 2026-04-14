import torch
import torch.nn as nn
import json

class LinearRegression(nn.Module):
    def __init__(self):
        super(LinearRegression, self).__init__()
        self.linear = nn.Linear(18, 12)  # 24 input features and 12 output

    def forward(self, x):
        return self.linear(x)


class RobotPredictionNN:
    def __init__(self, model_path='nn_model.pth'):
        # Instantiate the model
        self.model = LinearRegression()

        # Load the saved state dictionary into the model
        self.model.load_state_dict(torch.load(model_path))

        # Set the model to evaluation mode (important for inference)
        self.model.eval()

        self.prediction = None

    def predict(self, values):
        # Prepare the inputs to the NN model
        nn_input = torch.tensor(values)

        # Make the prediction with the neural network
        with torch.no_grad():  # No need to compute gradients during inference
            self.prediction = self.model(nn_input) #.item()
    
    def get_prediction(self):
        return self.prediction

if __name__ == "__main__":
    # Initialize the incubator prediction model
    # This will load the neural network model from 'temperature_prediction_model.pth'
    model_path = 'nn_model.pth'
    incubator_nn = RobotPredictionNN(model_path)

    # Provide a set of inputs: 
    # Tb: the current box temperature,
    # Tr: the room temperature,
    # H_h: the heater state (1 for ON, 0 for OFF)
    current_Tb = 25.0  # Current box temperature (in °C)
    room_Temp = 22.0   # Room temperature (in °C)
    heater_state = 1   # Heater state (1: ON, 0: OFF)


    with open(f"nn_folder/nn_test_data/trajectories_{100}.json", "r") as file:
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

    test_input = x[0]
    # Predict the next box temperature based on the input data
    incubator_nn.predict(test_input)

    # Retrieve and print the predicted temperature
    model_prediction = incubator_nn.get_prediction()
    if model_prediction is None:
        raise Exception("Model prediction cannot be none")
    
    model_prediction = model_prediction.tolist()
    for i in range(len(model_prediction)):
        model_prediction[i] = round(model_prediction[i], 3)

    actual = y[1]
    for i in range(len(actual)):
        actual[i] = round(actual[i], 3)
    print("Predicted:", model_prediction)
    print("Actual:", actual)