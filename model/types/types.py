import numpy as np
from enum import IntEnum
from dataclasses import dataclass

class LegPosition(IntEnum):
    FL = 0  # Front Left
    FR = 1  # Front Right
    BL = 2  # Back Left
    BR = 3  # Back Right

class LegPart(IntEnum):
    SHOULDER = 0
    ELBOW = 1
    HIP = 2

class BehaviorState(IntEnum):
    REST = 0
    MOVE = 1
    CALIBRATE = 2
    STAND = 3

@dataclass
class GyroData():
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

@dataclass
class RobotDogState():
    horizontal_velocity = np.array([0.0, 0.0])
    yaw_rate: float = 0.0
    height: float = 1
    behavior_state: BehaviorState = BehaviorState.REST
    gyro_data: GyroData