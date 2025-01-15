from enum import IntEnum
from utils.ConfigHelper import ConfigHelper

legs_motors_config = ConfigHelper.get_section("motors_legs")

class Motor(IntEnum):
    # identifies the corresponding pin location with the motor location
    FL_HIP = legs_motors_config.getint("FL_HIP")
    FL_SHOULDER = legs_motors_config.getint("FL_SHOULDER")
    FL_ELBOW = legs_motors_config.getint("FL_ELBOW")

    BL_HIP = legs_motors_config.getint("BL_HIP")
    BL_SHOULDER = legs_motors_config.getint("BL_SHOULDER")
    BL_ELBOW = legs_motors_config.getint("BL_ELBOW")

    FR_HIP = legs_motors_config.getint("FR_HIP")
    FR_SHOULDER = legs_motors_config.getint("FR_SHOULDER")
    FR_ELBOW = legs_motors_config.getint("FR_ELBOW")

    BR_HIP = legs_motors_config.getint("BR_HIP")
    BR_SHOULDER = legs_motors_config.getint("BR_SHOULDER")
    BR_ELBOW = legs_motors_config.getint("BR_ELBOW")


if __name__ == "__main__":
    for motor in Motor:
        print(f"{motor.name}: {motor.value}")