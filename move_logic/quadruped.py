import math
import threading
import numpy as np
from transforms3d.euler import euler2mat

from .types.leg import LegPosition, LegPart
from .hardware.Motor import Motor
from .hardware.ServoKitSingleton import ServoKitSingleton
from .LegController import LegController
from .motion_generator import generate_motion
from .kinematics import inverse_kinematics
from .State import RobotDogState, BehaviorState

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
        self.state = RobotDogState()
        self.moving_thread = threading.Thread()

    def get_angle(self, leg_postion: LegPosition, leg_part: LegPart):
        self.legs[leg_postion].get_angle(leg_part)

    def set_angle(self, leg_postion: LegPosition, leg_part: LegPart, degrees: float):
        self.legs[leg_postion].set_angle(leg_part, degrees)

    def set_leg_angle(self, leg_position: LegPosition, shoulder_angle: float, elbow_angle: float, hip_angle: float):
        self.legs[leg_position].set_shoulder_angle(shoulder_angle)
        self.legs[leg_position].set_elbow_angle(elbow_angle)
        self.legs[leg_position].set_hip_angle(hip_angle)
    
    def set_four_legs_angle(self, shoulder_angle: float, elbow_angle: float, hip_angle: float):
        for leg_position, leg_controller in self.legs.items():
            leg_controller.set_shoulder_angle(shoulder_angle)
            leg_controller.set_elbow_angle(elbow_angle)
            leg_controller.set_hip_angle(hip_angle)
            print(f"Set angles for {leg_position}: "
                f"Shoulder={shoulder_angle}, Elbow={elbow_angle}, Hip={hip_angle}")


    
    def calibrate(self):
        self.set_four_legs_angle(180, 30, 90)


    def standup(self):
        x, y, z = (0, -15, 0)
        theta_shoulder, theta_elbow, theta_hip = inverse_kinematics(x=x, y=y, z=z, a1=self.upper_leg_length, a2=self.lower_leg_length)
        self.set_four_legs_angle(theta_shoulder, theta_elbow, theta_hip)

    def move(self):
        index = 0

        # Generate footstep
        motion = generate_motion()

        while self.state.behavior_state == BehaviorState.MOVE:
            x_velocity, z_velocity = self.state.horizontal_velocity
            yaw_rate = self.state.yaw_rate
            y_height = self.state.height

            # 動態計算步態比例
            trajectory = motion * np.array([x_velocity, z_velocity, y_height])[:, None]

            # calculate rotation
            if not math.isclose(yaw_rate, 0):
                theta_yaw = math.radians(yaw_rate)
                # 構造旋轉矩陣
                R = euler2mat(0, 0, theta_yaw)

            x, z, y = trajectory  # 分解軌跡到 x, z, y
            i1 = index % 40
            i2 = (index + 20) % 40

            theta_shoulder_FL, theta_elbow_FL, theta_hip_FL = inverse_kinematics(
                x=x[i1], y=y[i1], z=z[i1], a1=self.upper_leg_length, a2=self.lower_leg_length
            )
            theta_shoulder_FR, theta_elbow_FR, theta_hip_FR = inverse_kinematics(
                x=x[i2], y=y[i2], z=z[i2], a1=self.upper_leg_length, a2=self.lower_leg_length
            )
            theta_shoulder_BL, theta_elbow_BL, theta_hip_BL = inverse_kinematics(
                x=x[i2], y=y[i2], z=z[i2], a1=self.upper_leg_length, a2=self.lower_leg_length
            )
            theta_shoulder_BR, theta_elbow_BR, theta_hip_BR = inverse_kinematics(
                x=x[i1], y=y[i1], z=z[i1], a1=self.upper_leg_length, a2=self.lower_leg_length
            )

            # 設定角度到對應的腿部控制器
            self.set_leg_angle(LegPosition.FL, theta_shoulder_FL, theta_elbow_FL, theta_hip_FL)
            self.set_leg_angle(LegPosition.FR, theta_shoulder_FR, theta_elbow_FR, theta_hip_FR)
            self.set_leg_angle(LegPosition.BL, theta_shoulder_BL, theta_elbow_BL, theta_hip_BL)
            self.set_leg_angle(LegPosition.BR, theta_shoulder_BR, theta_elbow_BR, theta_hip_BR)

            index += 1

            if np.allclose(self.state.horizontal_velocity, [0, 0]):
                print("Stopping robot...")
                self.state.behavior_state = BehaviorState.REST


    def run(self, command: RobotDogState=None):
        """主狀態機控制流程"""
        if command:
            self.update_state(command)

        # 根據行為狀態執行對應行為
        if self.state.behavior_state == BehaviorState.REST:
            if self.moving_thread.is_alive():
                self.moving_thread.join()
            self.standup()

        elif self.state.behavior_state == BehaviorState.MOVE:
            if not self.moving_thread.is_alive():
                print("Start moving thread...")
                self.moving_thread = threading.Thread(target=self.move)
                self.moving_thread.start()

        elif self.state.behavior_state == BehaviorState.CALIBRATE:
            if self.moving_thread.is_alive():
                self.moving_thread.join()
            self.calibrate()

    def update_state(self, command: RobotDogState):
        """根據指令更新狀態"""
        self.state.horizontal_velocity = command.horizontal_velocity
        self.state.yaw_rate = command.yaw_rate
        self.state.height = command.height
        self.state.behavior_state = command.behavior_state