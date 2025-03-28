from typing_extensions import Optional
import numpy as np
from enum import IntEnum
from typing import NamedTuple
from dataclasses import dataclass, field

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

class Position(NamedTuple):
    x: float
    y: float
    z: float

@dataclass
class GyroData():
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

@dataclass
class FootPositions:
    FL: Position
    FR: Position
    BL: Position
    BR: Position

@dataclass
class RobotDogState():
    foot_current_positions: Optional[FootPositions] = None
    gyro_data: Optional[GyroData] = None
    horizontal_velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    yaw_rate: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    height: float = 1
    behavior_state: BehaviorState = BehaviorState.REST

@dataclass
class MotionCommand:
    horizontal_velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    yaw_rate: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    height: float = 1
    behavior_state: BehaviorState=BehaviorState.REST
    is_gyro_activated: bool = False
