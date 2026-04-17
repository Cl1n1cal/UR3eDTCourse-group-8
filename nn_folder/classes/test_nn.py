import torch
import torch.nn as nn
import json
import numpy as np
from enum import Enum

class ModelType(Enum):
    SINGLE = 1
    MULTI = 2

class LinearRegressionSingle(nn.Module):
    def __init__(self):
        super(LinearRegressionSingle, self).__init__()
        self.linear = nn.Linear(21, 12)  # 24 input features and 12 output

    def forward(self, x):
        return self.linear(x)
    
    def get_name(self):
        return "LinearRegressionSingle"
    
    def get_result_file_name(self) -> str:
        return "model_single_results_new"

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
    
    def get_name(self):
        return "LinearRegressionMulti"
    
    def get_result_file_name(self) -> str:
        return "model_multi_results"

class RegressionFactory():

    @staticmethod
    def create_single():
        return LinearRegressionSingle()

    @staticmethod
    def create_multi():
        return LinearRegressionMulti()


class RobotPredictionNN:
    def __init__(self, model_path, model_t):
        # Instantiate the model
        if model_t == ModelType.SINGLE:
            self.model = RegressionFactory.create_single()
        else:
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
    
    def get_name(self):
        return self.model.get_name()
    
    def get_result_file_name(self) -> str:
        return self.model.get_result_file_name()


if __name__ == "__main__":
    # Initialize the model
    model_path_multi = 'nn_folder/models/nn_model_multi_layer.pth'
    model_path_single = 'nn_folder/models/nn_model_single_layer_new.pth'
    model = RobotPredictionNN(model_path_single, ModelType.SINGLE)
    model_result_file_name = model.get_result_file_name()

    pos_err_list = []
    vel_err_list = []
    joint_err_list = [[] for _ in range(6)]

    for q in range(0, 21):
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
                
                tmp_x.append(single_step["max_vel"])
                tmp_x.append(single_step["max_acc"])
                tmp_x.append(single_step["time_step"])
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
                model.predict(input)

                # Retrieve the predicted results
                model_prediction = model.get_prediction()
                if model_prediction is None:
                    raise Exception("Model prediction cannot be none")
                
                model_prediction = model_prediction.tolist()
                for i in range(len(model_prediction)):
                    model_prediction[i] = round(model_prediction[i], 4)


                # Check model prediction is the same length as output
                assert len(output) == len(model_prediction), "Model prediction is not the same size as output"

                # Calculate the error for each of the joint pos.
                for i in range(6):

                    v1 = output[i]
                    v2 = model_prediction[i]

                    tmp = np.abs(v2 - v1)
                    pos_err_list.append(tmp)

                    # Append the error for each individual joint
                    joint_err_list[i].append(tmp)

                # Calculate the error for each of the vels
                for i in range(6,12):
                    v1 = output[i]
                    v2 = model_prediction[i]

                    tmp = np.abs(v2 - v1)
                    vel_err_list.append(tmp)
    
    # Compute the avg. pos. err and round to 4 decimals
    avg_pos_err = round(np.average(pos_err_list), 4)
    pos_err_list.sort()
    med_pos_err = round(np.mean(pos_err_list), 4)
    
    # Compute the avg. vel err and round to 4 decimals
    avg_vel_err = round(np.average(vel_err_list), 4)
    vel_err_list.sort()
    med_vel_err = round(np.mean(vel_err_list), 4)

    # Write the average err in pct. to a file
    with open(f"nn_folder/nn_benchmarking/{model_result_file_name}.json", "w") as f:
        model_dict = {}
        data = {}
        model_dict[model.get_name()] = data

        data[f"avg_pos_err:"] = f"{avg_pos_err} r"
        data[f"med_pos_err:"] = f"{med_pos_err} r"
        data[f"avg_vel_err:"] = f"{avg_vel_err} deg/s"
        data[f"med_vel_err:"] = f"{med_vel_err} deg/s"

        # Calculate the avg. pos error for individual joints
        for i in range(6):
            tmp = np.average(joint_err_list[i])
            tmp = round(tmp, 4)
            data[f"q_{i}_avg_pos_err:"] = f"{tmp} r"
        
        json.dump(model_dict, f, indent=4)