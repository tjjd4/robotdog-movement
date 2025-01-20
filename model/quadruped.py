import math
import time
import threading
import numpy as np

from .types.leg import LegPosition, LegPart
from .hardware.Motor import Motor
from .hardware.ServoKitSingleton import ServoKitSingleton
from .LegController import LegController
from .MotionGenerator import MotionGenerator
from .kinematics import inverse_kinematics
from .State import RobotDogState, BehaviorState

from utils.ConfigHelper import ConfigHelper
from utils.math import get_plane_from_points, turn_points_with_euler_radians

class Robotdog:
    def __init__(self) -> None:
        self.robotdog_config = ConfigHelper.get_section("robotdog_parameters")
        self.legs_config = ConfigHelper.get_section("motors_legs")
        self.upper_leg_length = self.robotdog_config.getfloat("upper_leg_length")
        self.lower_leg_length = self.robotdog_config.getfloat("lower_leg_length")
        self.delay_time = self.robotdog_config.getfloat("delay_time")
        self.legs: dict[LegPosition, LegController] = {
            LegPosition.FL: LegController(Motor.FL_SHOULDER, Motor.FL_ELBOW, Motor.FL_HIP, FB_is_opposited=self.legs_config.getboolean("FB_FL_is_opposited"), LR_is_opposited=self.legs_config.getboolean("LR_FL_is_opposited")),
            LegPosition.FR: LegController(Motor.FR_SHOULDER, Motor.FR_ELBOW, Motor.FR_HIP, FB_is_opposited=self.legs_config.getboolean("FB_FR_is_opposited"), LR_is_opposited=self.legs_config.getboolean("LR_FR_is_opposited")),
            LegPosition.BL: LegController(Motor.BL_SHOULDER, Motor.BL_ELBOW, Motor.BL_HIP, FB_is_opposited=self.legs_config.getboolean("FB_BL_is_opposited"), LR_is_opposited=self.legs_config.getboolean("LR_BL_is_opposited")),
            LegPosition.BR: LegController(Motor.BR_SHOULDER, Motor.BR_ELBOW, Motor.BR_HIP, FB_is_opposited=self.legs_config.getboolean("FB_BR_is_opposited"), LR_is_opposited=self.legs_config.getboolean("LR_BR_is_opposited")),
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


    def adjust_for_turning(self, trajectory: np.ndarray, turning_factor: float) -> np.ndarray:
        """
        Adjust the step size for turning using the small-step, big-step method.

        Args:
            motion (np.ndarray): Original motion trajectory (shape: [3, n_steps]).
            turning_factor (float): Positive for turning right, negative for turning left.

        Returns:
            np.ndarray: Adjusted motion trajectory.
        """
        left_trajectory = trajectory.copy()
        right_trajectory = trajectory.copy()

        if turning_factor > 0:  # Turn Right
            # Left legs take bigger steps, right legs take smaller steps
            left_trajectory[0, :] *= (1 + turning_factor)  # Left legs bigger steps
            right_trajectory[0, :] *= (1 - turning_factor)  # Right legs smaller steps
        elif turning_factor < 0:  # Turn Left
            turning_factor = abs(turning_factor)
            # Right legs take bigger steps, left legs take smaller steps
            left_trajectory[0, :] *= (1 - turning_factor)  # Left legs smaller steps
            right_trajectory[0, :] *= (1 + turning_factor)  # Right legs bigger steps

        return left_trajectory, right_trajectory

    def move(self):
        index = 0

        # Generate footstep
        motion = MotionGenerator.generate_motion()
        last_loop = time.time()

        while self.state.behavior_state == BehaviorState.MOVE:
            now = time.time()
            if now - last_loop < self.delay_time:
                continue
            last_loop = time.time()
            x_velocity, z_velocity = self.state.horizontal_velocity
            yaw_rate = self.state.yaw_rate
            y_height = self.state.height

            # 動態計算步態比例
            trajectory = motion * np.array([x_velocity, z_velocity, y_height])[:, None]

            if yaw_rate != 0.0:
                left_trajectory, right_trajectory = self.adjust_for_turning(trajectory, self.state.yaw_rate)
            else:
                left_trajectory = trajectory.copy()
                right_trajectory = trajectory.copy()
            left_x, left_z, left_y = left_trajectory
            right_x, right_z, right_y = right_trajectory
            x, z, y = trajectory  # 分解軌跡到 x, z, y
            i1 = index % 40
            i2 = (index + 20) % 40

            theta_shoulder_FL, theta_elbow_FL, theta_hip_FL = inverse_kinematics(
                x=left_x[i1]+3, y=left_y[i1], z=left_z[i1], a1=self.upper_leg_length, a2=self.lower_leg_length
            )
            theta_shoulder_FR, theta_elbow_FR, theta_hip_FR = inverse_kinematics(
                x=right_x[i2]+3, y=right_y[i2], z=right_z[i2], a1=self.upper_leg_length, a2=self.lower_leg_length
            )
            theta_shoulder_BL, theta_elbow_BL, theta_hip_BL = inverse_kinematics(
                x=left_x[i2], y=left_y[i2], z=left_z[i2], a1=self.upper_leg_length, a2=self.lower_leg_length
            )
            theta_shoulder_BR, theta_elbow_BR, theta_hip_BR = inverse_kinematics(
                x=right_x[i1], y=right_y[i1], z=right_z[i1], a1=self.upper_leg_length, a2=self.lower_leg_length
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