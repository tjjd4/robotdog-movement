from enum import IntEnum

class Motor(IntEnum):
    # identifies the corresponding pin location with the motor location
    FL_HIP = 1
    FL_SHOULDER = 2
    FL_ELBOW = 3
    BL_HIP = 4
    BL_SHOULDER = 5
    BL_ELBOW = 6
    FR_HIP = 7
    FR_SHOULDER = 8
    FR_ELBOW = 9
    BR_HIP = 10
    BR_SHOULDER = 11
    BR_ELBOW = 12