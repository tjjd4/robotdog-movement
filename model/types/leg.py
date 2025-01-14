from enum import IntEnum

class LegPosition(IntEnum):
    FL = 0  # Front Left
    FR = 1  # Front Right
    BL = 2  # Back Left
    BR = 3  # Back Right

class LegPart(IntEnum):
    SHOULDER = 0
    ELBOW = 1
    HIP = 2