import torch
import torch.nn as nn
import json
import numpy as np
from enum import Enum
from nn_folder.classes.linear_regression_single import LinearRegressionSingle
from nn_folder.classes.linear_regression_multi import LinearRegressionMulti

class RobotPredictionNN:
    def __init__(self):
        # Instantiate the model
        self.model = LinearRegressionMulti()
        self.prediction = []

    def setup(self, model_path):
        # Load the saved state dictionary into the model
        # Use cpu as default for now
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))

        # Set the model to evaluation mode (important for inference)
        self.model.eval()

    def predict(self, values):
        # Prepare the inputs to the NN model
        nn_input = torch.tensor(values)

        # Make the prediction with the neural network
        with torch.no_grad():  # No need to compute gradients during inference
            self.prediction = self.model(nn_input)
    
    def get_prediction(self):
        return self.prediction
    
    def get_name(self):
        return self.model.get_name()
    
    def get_result_file_name(self) -> str:
        return self.model.get_result_file_name()