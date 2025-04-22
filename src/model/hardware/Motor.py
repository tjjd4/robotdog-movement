from enum import IntEnum
from src.utils.ConfigHelper import ConfigHelper

legs_motors_config = ConfigHelper.get_section("motors_legs")

class Motor(IntEnum):
    # identifies the corresponding pin location with the motor location
    FL_HIP = legs_motors_config.getint("FL_HIP", fallback=1)
    FL_SHOULDER = legs_motors_config.getint("FL_SHOULDER", fallback=2)
    FL_ELBOW = legs_motors_config.getint("FL_ELBOW", fallback=3)

    BL_HIP = legs_motors_config.getint("BL_HIP", fallback=4)
    BL_SHOULDER = legs_motors_config.getint("BL_SHOULDER", fallback=5)
    BL_ELBOW = legs_motors_config.getint("BL_ELBOW", fallback=6)

    FR_HIP = legs_motors_config.getint("FR_HIP", fallback=7)
    FR_SHOULDER = legs_motors_config.getint("FR_SHOULDER", fallback=8)
    FR_ELBOW = legs_motors_config.getint("FR_ELBOW", fallback=9)

    BR_HIP = legs_motors_config.getint("BR_HIP", fallback=10)
    BR_SHOULDER = legs_motors_config.getint("BR_SHOULDER", fallback=11)
    BR_ELBOW = legs_motors_config.getint("BR_ELBOW", fallback=12)


if __name__ == "__main__":
    for motor in Motor:
        print(f"{motor.name}: {motor.value}")
