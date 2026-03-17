from enum import Enum
import numpy as np

pi: float = np.pi
zero: float = 0.0
step_size = 0.05 # 0.05 seconds (default)

class State(Enum):
    IDLE = "Idle"
    RUNNING = "Running"
