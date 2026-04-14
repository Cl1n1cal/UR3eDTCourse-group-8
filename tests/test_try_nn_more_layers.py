import torch
import torch.nn as nn
import json
import numpy as np

class LinearRegressionSingle(nn.Module):
    def __init__(self):
        super(LinearRegressionSingle, self).__init__()
        self.linear = nn.Linear(18, 12)  # 24 input features and 12 output

    def forward(self, x):
        return self.linear(x)

class LinearRegressionMulti(nn.Module):
    def __init__(self):
        super(LinearRegressionMulti, self).__init__()
        
        self.net = nn.Sequential(
            nn.Linear(18, 64),   # hidden layer 1
            nn.ReLU(),
            nn.Linear(64, 32),   # hidden layer 2
            nn.ReLU(),
            nn.Linear(32, 12)    # output layer
        )

    def forward(self, x):
        return self.net(x)

class RegressionFactory():

    @staticmethod
    def create_single():
        return LinearRegressionSingle()

    @staticmethod
    def create_multi():
        return LinearRegressionMulti()


class RobotPredictionNN:
    def __init__(self, model_path='nn_model_more_layers.pth'):
        # Instantiate the model
        self.model = RegressionFactory.create_multi()

        # Load the saved state dictionary into the model
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))

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
    model_path = 'nn_model_more_layers.pth'
    incubator_nn = RobotPredictionNN(model_path)

    err_list = []

    for q in range(0, 33):
        print("q:", q)
        with open(f"nn_folder/nn_test_data/trajectories_{q}.json", "r") as file:
            data = json.load(file)
    
        for d in data.keys():
            x = []
            y = []
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
            
            # Adjust x and y so that the results match
            x = x[0:-1] # Get all but the last element of x
            y = y[1::] # Get all but the first element of y

            # Check that x and y are the same size
            assert len(x) == len(y), "x and y are not the same size"

            # Loop through the lists 
            for i in range(len(x)):
                input = x[i]
                output = y[i]

                # Round the output
                for i in range(len(output)):
                    output[i] = round(output[i], 4)

                # Predict the next position and velocities
                incubator_nn.predict(input)

                # Retrieve the predicted results
                model_prediction = incubator_nn.get_prediction()
                if model_prediction is None:
                    raise Exception("Model prediction cannot be none")
                
                model_prediction = model_prediction.tolist()
                for i in range(len(model_prediction)):
                    model_prediction[i] = round(model_prediction[i], 4)


                # Check model prediction is the same length as output
                assert len(output) == len(model_prediction), "Model prediction is not the same size as output"

                # Calculate the error for each of the joint and vel predictions and convert it to pct.
                for i in range(len(output)):
                    tmp = np.abs(np.abs(model_prediction[i]) - np.abs(output[i]))
                    err_list.append(tmp * 100)
    
    avg_err = np.average(err_list)

    with open("nn_folder/nn_benchmarking/multi_layer_model.json", "w") as f:
        data = {}
        data["Average_error"] = f"{avg_err}%"
        json.dump(data, f, indent=4)         