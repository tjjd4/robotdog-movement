import numpy as np
from .types.BehaviorState import BehaviorState

class MotionCommand:
    def __init__(
            self, 
            horizontal_velocity=np.array([0.0, 0.0]),
            yaw_rate: float=0.0,
            height: float=1,
            behavior_state: BehaviorState=BehaviorState.REST
        ):
        self.horizontal_velocity = horizontal_velocity
        self.yaw_rate = yaw_rate
        self.height = height
        self.behavior_state = behavior_state