import torch.nn as nn

class LinearRegressionMulti(nn.Module):
    def __init__(self):
        super(LinearRegressionMulti, self).__init__()
        
        self.net = nn.Sequential(
            nn.Linear(21, 128),
            nn.ReLU(),
            nn.Linear(128, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 12)
        )

    def forward(self, x):
        return self.net(x)