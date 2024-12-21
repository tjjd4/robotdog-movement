from adafruit_servokit import ServoKit
from enum import IntEnum
import math
import bezier
import numpy as np

from types.leg import LegPosition, LegPart
from hardware.Motor import Motor
from LegController import LegController
from motion_generator import generate_motion
from kinematics import inverse_kinematics
from hardware.ServoKitSingleton import ServoKitSingleton


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

    def get_angle(self, motor_id: Motor):
        return self.kit.servo[motor_id].angle

    def set_angle(self, motor_id: Motor, degrees: int):
        self.kit.servo[motor_id].angle = degrees

    def get_angle_by_module(self, leg_postion: LegPosition, leg_part: LegPart, degrees: int):
        self.legs[leg_postion].get_angle_by_module(leg_part, degrees)

    def set_angle_by_module(self, leg_postion: LegPosition, leg_part: LegPart, degrees: int):
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
        self.set_angle(Motor.FL_HIP, 90)
        self.set_angle(Motor.FL_SHOULDER, 0)
        self.set_angle(Motor.FL_ELBOW, 90)
        self.set_angle(Motor.FR_HIP, 90)
        self.set_angle(Motor.FR_SHOULDER, 180)
        self.set_angle(Motor.FR_ELBOW, 90)

        self.set_angle(Motor.BL_HIP, 90)
        self.set_angle(Motor.BL_SHOULDER, 0)
        self.set_angle(Motor.BL_ELBOW, 90)
        self.set_angle(Motor.BR_HIP, 90)
        self.set_angle(Motor.BR_SHOULDER, 180)
        self.set_angle(Motor.BR_ELBOW, 90)


    def calibrate_by_inverse_positioning(self):
        x, y, z = (0, -15, 0)
        self.inverse_positioning(Motor.FL_SHOULDER,Motor.FL_ELBOW,x,y,z=z,hip=Motor.FL_HIP,right=False)
        self.inverse_positioning(Motor.FR_SHOULDER,Motor.FR_ELBOW,x,y,z=z,hip=Motor.FR_HIP,right=True)
        self.inverse_positioning(Motor.BL_SHOULDER,Motor.BL_ELBOW,x,y,z=z,hip=Motor.BL_HIP,right=False)
        self.inverse_positioning(Motor.BR_SHOULDER,Motor.BR_ELBOW,x,y,z=z,hip=Motor.BR_HIP,right=True)

    def inverse_positioning(self, shoulder, elbow, x, y, z=0, hip=None, right=True):
        '''
        Positions the end effector at a given position based on cartesian coordinates in 
        centimeter units and with respect to the should motor of the
        :param shoulder: motor id used for the shoulder
        :param elbow: motor id used for the elbow
        :param x: cartesian x with respect to shoulder motor (forward/back)
        :param y: cartesian y with respect to shoulder motor (up/down)
        :param z: cartesian z with respect to shoulder motor (left/right)
        :param hip: motor id used for the hip
        :param right: a boolean that flips the logic for left and right side to properly map "forward direction"
        :return: a list containing the appropriate angle for the shoulder and elbow
        '''
        L=0
        # 距離
        y_prime = -math.sqrt((z+L)**2 + y**2)
        thetaz = math.atan2(z+L,abs(y))-math.atan2(L,abs(y_prime))

        elbow_offset = 0
        shoulder_offset = 0
        a1 = self.upper_leg_length
        a2 = self.lower_leg_length

        c2 = (x**2+y_prime**2-a1**2-a2**2)/(2*a1*a2)
        s2 = math.sqrt(1-c2**2)
        theta2 = math.atan2(s2,c2)
        c2 = math.cos(theta2)
        s2 = math.sin(theta2)

        c1 = (x*(a1+(a2*c2)) + y_prime*(a2*s2))/(x**2+y_prime**2)
        s1 = (y_prime*(a1+(a2*c2)) - x*(a2*s2))/(x**2+y_prime**2)
        theta1 = math.atan2(s1,c1)
        # generate positions with respect to robot motors
        theta_shoulder = -theta1
        theta_elbow = theta_shoulder - theta2
        theta_hip = 0
        if right:
            theta_shoulder = self.rad_to_degree(theta_shoulder) + shoulder_offset
            theta_elbow = self.rad_to_degree(theta_elbow) + elbow_offset
            if hip:
                theta_hip = 90 - self.rad_to_degree(thetaz)
        else:
            theta_shoulder = 180 - self.rad_to_degree(theta_shoulder) - shoulder_offset
            theta_elbow = 180 - self.rad_to_degree(theta_elbow) - elbow_offset
            if hip:
                theta_hip = 90 + self.rad_to_degree(thetaz)
        self.set_angle(shoulder, theta_shoulder)
        self.set_angle(elbow, theta_elbow)
        if hip:
            self.set_angle(hip, theta_hip)
        # print("theta shoulder:",theta_shoulder,"\ttheta_elbow:",theta_elbow)
        return [theta_shoulder, theta_elbow]

    def leg_position(self, leg_id, x, y, z=0):
        """
        wrapper for inverse position that makes it easier to control each leg for making fixed paths
        :param led_id: string for the leg to be manipulated
        :param x: cartesian x with respect to shoulder motor (forward/back)
        :param y: cartesian y with respect to shoulder motor (up/down)
        :param z: cartesian z with respect to shoulder motor (left/right)
        """
        if leg_id == 'FL':
            self.inverse_positioning(Motor.FL_SHOULDER, Motor.FL_ELBOW, x, y, z=z, hip=Motor.FL_HIP, right=False)
        if leg_id == 'FR':
            self.inverse_positioning(Motor.FR_SHOULDER, Motor.FR_ELBOW, x, y, z=z, hip=Motor.FR_HIP, right=True)
        if leg_id == 'BL':
            self.inverse_positioning(Motor.BL_SHOULDER, Motor.BL_ELBOW, x, y, right=False)
        if leg_id == 'BR':
            self.inverse_positioning(Motor.BR_SHOULDER, Motor.BR_ELBOW, x, y, right=True)
    
    def move(self, controller=None):
        """
        Walks around based on the controller inputted momentum
        :param controller: the controller that is called to determine the robot momentum
        :returns: None, enters an infinite loop 
        """
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
            self.inverse_positioning(Motor.FR_SHOULDER,Motor.FR_ELBOW,x[i1],y[i1],z=z[i1],hip=Motor.FR_HIP,right=True)
            self.inverse_positioning(Motor.BR_SHOULDER,Motor.BR_ELBOW,x[i2],y[i2],right=True)
            self.inverse_positioning(Motor.FL_SHOULDER,Motor.FL_ELBOW,x[i2],y[i2],z=-z[i2],hip=Motor.FL_HIP,right=False)
            self.inverse_positioning(Motor.BL_SHOULDER,Motor.BL_ELBOW,x[i1],y[i1],right=False)
            index += 1

    def move_by_module(self, controller=None):

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

            self.set_angle_by_module(LegPosition.FL, LegPart.SHOULDER, theta_shoulder_FL)
            self.set_angle_by_module(LegPosition.FL, LegPart.ELBOW, theta_elbow_FL)
            self.set_angle_by_module(LegPosition.FL, LegPart.HIP, theta_hip_FL)

            self.set_angle_by_module(LegPosition.FR, LegPart.SHOULDER, theta_shoulder_FR)
            self.set_angle_by_module(LegPosition.FR, LegPart.ELBOW, theta_elbow_FR)
            self.set_angle_by_module(LegPosition.FR, LegPart.HIP, theta_hip_FR)

            self.set_angle_by_module(LegPosition.BL, LegPart.SHOULDER, theta_shoulder_BL)
            self.set_angle_by_module(LegPosition.BL, LegPart.ELBOW, theta_elbow_BL)
            self.set_angle_by_module(LegPosition.BL, LegPart.HIP, theta_hip_BL)

            self.set_angle_by_module(LegPosition.BR, LegPart.SHOULDER, theta_shoulder_BR)
            self.set_angle_by_module(LegPosition.BR, LegPart.ELBOW, theta_elbow_BR)
            self.set_angle_by_module(LegPosition.BR, LegPart.HIP, theta_hip_BR)

            index += 1