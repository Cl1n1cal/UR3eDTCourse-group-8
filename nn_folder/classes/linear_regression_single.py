import torch.nn as nn

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