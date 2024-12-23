import math
import numpy as np

from .types.leg import LegPosition, LegPart
from .hardware.Motor import Motor
from .hardware.ServoKitSingleton import ServoKitSingleton
from .LegController import LegController
from .motion_generator import generate_motion
from .kinematics import inverse_kinematics


class Robotdog:
    def __init__(self) -> None:
        self.upper_leg_length = 10
        self.lower_leg_length = 10
        self.legs: dict[LegPosition, LegController] = {
            LegPosition.FL: LegController(Motor.FL_SHOULDER, Motor.FL_ELBOW, Motor.FL_HIP, is_opposited=False),
            LegPosition.FR: LegController(Motor.FR_SHOULDER, Motor.FR_ELBOW, Motor.FR_HIP, is_opposited=True),
            LegPosition.BL: LegController(Motor.BL_SHOULDER, Motor.BL_ELBOW, Motor.BL_HIP, is_opposited=False),
            LegPosition.BR: LegController(Motor.BR_SHOULDER, Motor.BR_ELBOW, Motor.BR_HIP, is_opposited=True),
        }
        self.kit = ServoKitSingleton.get_instance()

    def get_angle(self, leg_postion: LegPosition, leg_part: LegPart, degrees: int):
        self.legs[leg_postion].get_angle_by_module(leg_part, degrees)

    def set_angle(self, leg_postion: LegPosition, leg_part: LegPart, degrees: int):
        self.legs[leg_postion].set_angle_by_module(leg_part, degrees)

    def rad_to_degree(self, rad):
        return rad*180/math.pi
    
    def standup(self):
        self.legs[LegPosition.FL].set_shoulder_angle(0)
        self.legs[LegPosition.FL].set_elbow_angle(90)
        self.legs[LegPosition.FL].set_hip_angle(90)

        self.legs[LegPosition.FR].set_shoulder_angle(180)
        self.legs[LegPosition.FR].set_elbow_angle(90)
        self.legs[LegPosition.FR].set_hip_angle(90)

        self.legs[LegPosition.BL].set_shoulder_angle(0)
        self.legs[LegPosition.BL].set_elbow_angle(90)
        self.legs[LegPosition.BL].set_hip_angle(90)

        self.legs[LegPosition.BR].set_shoulder_angle(180)
        self.legs[LegPosition.BR].set_elbow_angle(90)
        self.legs[LegPosition.BR].set_hip_angle(90)

    def inverse_position_to_motor_degrees(self, x: float, y: float, z: float):
        return inverse_kinematics(x, y, z, self.upper_leg_length, self.lower_leg_length)
    
    def calibrate(self):
        self.set_angle(LegPosition.FL, LegPart.SHOULDER, 180)
        self.set_angle(LegPosition.FL, LegPart.ELBOW, 90)
        self.set_angle(LegPosition.FL, LegPart.HIP, 90)

        self.set_angle(LegPosition.FR, LegPart.SHOULDER, 180)
        self.set_angle(LegPosition.FR, LegPart.ELBOW, 90)
        self.set_angle(LegPosition.FR, LegPart.HIP, 90)

        self.set_angle(LegPosition.BL, LegPart.SHOULDER, 180)
        self.set_angle(LegPosition.BL, LegPart.ELBOW, 90)
        self.set_angle(LegPosition.BL, LegPart.HIP, 90)

        self.set_angle(LegPosition.BR, LegPart.SHOULDER, 180)
        self.set_angle(LegPosition.BR, LegPart.ELBOW, 90)
        self.set_angle(LegPosition.BR, LegPart.HIP, 90)


    def calibrate_by_inverse_positioning(self):
        x, y, z = (0, -15, 0)
        theta_shoulder_FL, theta_elbow_FL, theta_hip_FL = inverse_kinematics(x=x, y=y, z=z, a1=self.upper_leg_length, a2=self.lower_leg_length)
        theta_shoulder_FR, theta_elbow_FR, theta_hip_FR = inverse_kinematics(x=x, y=y, z=z, a1=self.upper_leg_length, a2=self.lower_leg_length)

        theta_shoulder_BL, theta_elbow_BL, theta_hip_BL = inverse_kinematics(x=x, y=y, z=z, a1=self.upper_leg_length, a2=self.lower_leg_length)
        theta_shoulder_BR, theta_elbow_BR, theta_hip_BR = inverse_kinematics(x=x, y=y, z=z, a1=self.upper_leg_length, a2=self.lower_leg_length)

        self.set_angle(LegPosition.FL, LegPart.SHOULDER, theta_shoulder_FL)
        self.set_angle(LegPosition.FL, LegPart.ELBOW, theta_elbow_FL)
        self.set_angle(LegPosition.FL, LegPart.HIP, theta_hip_FL)

        self.set_angle(LegPosition.FR, LegPart.SHOULDER, theta_shoulder_FR)
        self.set_angle(LegPosition.FR, LegPart.ELBOW, theta_elbow_FR)
        self.set_angle(LegPosition.FR, LegPart.HIP, theta_hip_FR)

        self.set_angle(LegPosition.BL, LegPart.SHOULDER, theta_shoulder_BL)
        self.set_angle(LegPosition.BL, LegPart.ELBOW, theta_elbow_BL)
        self.set_angle(LegPosition.BL, LegPart.HIP, theta_hip_BL)

        self.set_angle(LegPosition.BR, LegPart.SHOULDER, theta_shoulder_BR)
        self.set_angle(LegPosition.BR, LegPart.ELBOW, theta_elbow_BR)
        self.set_angle(LegPosition.BR, LegPart.HIP, theta_hip_BR)


    def move(self, controller=None):

        momentum = np.asarray([0,0,1,0],dtype=np.float32)  # 前三個值 x(前後), z(左右), y(上下) 為步伐大小的縮放值, 第四個值不為 0 時結束動作
        index = 0
        
        # Generate footstep
        motion = generate_motion()

        close = False
        while not close:
            momentum = controller(momentum)
            tragectory = motion * momentum[:3, None]
            if momentum[3]:
                close = True
            x,z,y = tragectory
            # 
            i1 = index%40
            i2 = (index+20)%40 
            # Apply movement based movement
            theta_shoulder_FL, theta_elbow_FL, theta_hip_FL = inverse_kinematics(x=x[i1], y=y[i1], z=z[i1], a1=self.upper_leg_length, a2=self.lower_leg_length)
            theta_shoulder_FR, theta_elbow_FR, theta_hip_FR = inverse_kinematics(x=x[i2], y=y[i2], z=z[i2], a1=self.upper_leg_length, a2=self.lower_leg_length)

            theta_shoulder_BL, theta_elbow_BL, theta_hip_BL = inverse_kinematics(x=x[i2], y=y[i2], z=z[i2], a1=self.upper_leg_length, a2=self.lower_leg_length)
            theta_shoulder_BR, theta_elbow_BR, theta_hip_BR = inverse_kinematics(x=x[i1], y=y[i1], z=z[i1], a1=self.upper_leg_length, a2=self.lower_leg_length)

            self.set_angle(LegPosition.FL, LegPart.SHOULDER, theta_shoulder_FL)
            self.set_angle(LegPosition.FL, LegPart.ELBOW, theta_elbow_FL)
            self.set_angle(LegPosition.FL, LegPart.HIP, theta_hip_FL)

            self.set_angle(LegPosition.FR, LegPart.SHOULDER, theta_shoulder_FR)
            self.set_angle(LegPosition.FR, LegPart.ELBOW, theta_elbow_FR)
            self.set_angle(LegPosition.FR, LegPart.HIP, theta_hip_FR)

            self.set_angle(LegPosition.BL, LegPart.SHOULDER, theta_shoulder_BL)
            self.set_angle(LegPosition.BL, LegPart.ELBOW, theta_elbow_BL)
            self.set_angle(LegPosition.BL, LegPart.HIP, theta_hip_BL)

            self.set_angle(LegPosition.BR, LegPart.SHOULDER, theta_shoulder_BR)
            self.set_angle(LegPosition.BR, LegPart.ELBOW, theta_elbow_BR)
            self.set_angle(LegPosition.BR, LegPart.HIP, theta_hip_BR)

            index += 1