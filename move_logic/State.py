import numpy as np
from enum import IntEnum

from .types.BehaviorState import BehaviorState

class RobotDogState:
    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = 1
        self.behavior_state = BehaviorState.REST

