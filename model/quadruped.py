import time
from threading import Thread
from queue import LifoQueue
import numpy as np

from .types.types import LegPosition, LegPart, RobotDogState, BehaviorState, GyroData
from .hardware.Motor import Motor
from .LegController import LegController
from .GyroscopeController import GyroscopeController
from .MotionGenerator import MotionGenerator
from .kinematics import get_angle_from_position

from utils.ConfigHelper import ConfigHelper

class Robotdog:
    def __init__(self) -> None:
        self.robotdog_config = ConfigHelper.get_section("robotdog_parameters")
        self.movement_config = ConfigHelper.get_section("movement_parameters")
        self.legs_config = ConfigHelper.get_section("motors_legs")
        self.upper_leg_length = self.robotdog_config.getfloat("upper_leg_length")
        self.lower_leg_length = self.robotdog_config.getfloat("lower_leg_length")
        self.delay_time = self.movement_config.getfloat("delay_time")
        self.legs: dict[LegPosition, LegController] = {
            LegPosition.FL: LegController(Motor.FL_SHOULDER, Motor.FL_ELBOW, Motor.FL_HIP, FB_is_opposited=self.legs_config.getboolean("FB_FL_is_opposited"), LR_is_opposited=self.legs_config.getboolean("LR_FL_is_opposited")),
            LegPosition.FR: LegController(Motor.FR_SHOULDER, Motor.FR_ELBOW, Motor.FR_HIP, FB_is_opposited=self.legs_config.getboolean("FB_FR_is_opposited"), LR_is_opposited=self.legs_config.getboolean("LR_FR_is_opposited")),
            LegPosition.BL: LegController(Motor.BL_SHOULDER, Motor.BL_ELBOW, Motor.BL_HIP, FB_is_opposited=self.legs_config.getboolean("FB_BL_is_opposited"), LR_is_opposited=self.legs_config.getboolean("LR_BL_is_opposited")),
            LegPosition.BR: LegController(Motor.BR_SHOULDER, Motor.BR_ELBOW, Motor.BR_HIP, FB_is_opposited=self.legs_config.getboolean("FB_BR_is_opposited"), LR_is_opposited=self.legs_config.getboolean("LR_BR_is_opposited")),
        }
        self.gyro_queue = LifoQueue()
        self.gyroscope = GyroscopeController(gyro_queue=self.gyro_queue)
        self.state = RobotDogState()
        self.moving_thread = Thread()
        self.standing_thread = Thread()
        self.gyro_thread = Thread()
        self.is_gyroscope_running = False

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
            # print(f"Set angles for {leg_position}: "
            #     f"Shoulder={shoulder_angle}, Elbow={elbow_angle}, Hip={hip_angle}")


    
    def calibrate(self):
        self.set_four_legs_angle(180, 30, 90)


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
    
    def standup(self):
        last_loop = time.time()
        while self.state.behavior_state == BehaviorState.STAND:
            now = time.time()
            if now - last_loop < self.delay_time:
                continue
            last_loop = time.time()
            x, y, z = (0, -15, 0)

            gyro_data = None
            if self.is_gyroscope_running:
                gyro_data = self.state.gyro_data

            theta_shoulder_FL, theta_elbow_FL, theta_hip_FL = get_angle_from_position(
                x=x, y=y, z=z, legPosition=LegPosition.FL, gyro_data=gyro_data
            )
            theta_shoulder_FR, theta_elbow_FR, theta_hip_FR = get_angle_from_position(
                x=x, y=y, z=z, legPosition=LegPosition.FR, gyro_data=gyro_data
            )
            theta_shoulder_BL, theta_elbow_BL, theta_hip_BL = get_angle_from_position(
                x=x, y=y, z=z, legPosition=LegPosition.BL, gyro_data=gyro_data
            )
            theta_shoulder_BR, theta_elbow_BR, theta_hip_BR = get_angle_from_position(
                x=x, y=y, z=z, legPosition=LegPosition.BR, gyro_data=gyro_data
            )

            # 設定角度到對應的腿部控制器
            self.set_leg_angle(LegPosition.FL, theta_shoulder_FL, theta_elbow_FL, theta_hip_FL)
            self.set_leg_angle(LegPosition.FR, theta_shoulder_FR, theta_elbow_FR, theta_hip_FR)
            self.set_leg_angle(LegPosition.BL, theta_shoulder_BL, theta_elbow_BL, theta_hip_BL)
            self.set_leg_angle(LegPosition.BR, theta_shoulder_BR, theta_elbow_BR, theta_hip_BR)


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
            # x, z, y = trajectory  # 分解軌跡到 x, z, y
            i1 = index % 40
            i2 = (index + 20) % 40

            gyro_data = None
            if self.is_gyroscope_running:
                gyro_data = self.state.gyro_data

            theta_shoulder_FL, theta_elbow_FL, theta_hip_FL = get_angle_from_position(
                x=left_x[i1]+3, y=left_y[i1], z=left_z[i1], legPosition=LegPosition.FL, gyro_data=gyro_data
            )
            theta_shoulder_FR, theta_elbow_FR, theta_hip_FR = get_angle_from_position(
                x=right_x[i2]+3, y=right_y[i2], z=right_z[i2], legPosition=LegPosition.FR, gyro_data=gyro_data
            )
            theta_shoulder_BL, theta_elbow_BL, theta_hip_BL = get_angle_from_position(
                x=left_x[i2], y=left_y[i2], z=left_z[i2], legPosition=LegPosition.BL, gyro_data=gyro_data
            )
            theta_shoulder_BR, theta_elbow_BR, theta_hip_BR = get_angle_from_position(
                x=right_x[i1], y=right_y[i1], z=right_z[i1], legPosition=LegPosition.BR, gyro_data=gyro_data
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
        if self.state.behavior_state == BehaviorState.STAND:
            if self.moving_thread.is_alive():
                self.moving_thread.join()

            if not self.standing_thread.is_alive():
                print("Start standing thread...")
                self.standing_thread = Thread(target=self.standup, daemon=True)
                self.standing_thread.start()

        elif self.state.behavior_state == BehaviorState.MOVE:
            if self.standing_thread.is_alive():
                self.standing_thread.join()

            if not self.moving_thread.is_alive():
                print("Start moving thread...")
                self.moving_thread = Thread(target=self.move, daemon=True)
                self.moving_thread.start()

        elif self.state.behavior_state == BehaviorState.CALIBRATE:
            if self.moving_thread.is_alive():
                self.moving_thread.join()
            if self.standing_thread.is_alive():
                self.standing_thread.join()
            self.calibrate()

    def update_state(self, command: RobotDogState):
        """根據指令更新狀態"""
        self.state.horizontal_velocity = command.horizontal_velocity
        self.state.yaw_rate = command.yaw_rate
        self.state.height = command.height
        self.state.behavior_state = command.behavior_state

    def update_gyro_data(self):
        while self.is_gyroscope_running:
            if not self.gyro_queue.empty():
                gyro_data: GyroData = self.gyro_queue.get()
                self.state.gyro_data = gyro_data

            time.sleep(0.25)
    
    def activate_gyroscope(self):
        if not self.is_gyroscope_running:
            self.is_gyroscope_running = True
            if not self.gyroscope.is_running():
                self.gyroscope.start()
            if not self.gyro_thread.is_alive():
                self.gyro_thread = Thread(target=self.update_gyro_data)
            print("LOG: Gyroscope activated.")
        else:
            print("LOG: Gyroscope already activated!")
            
    
    def deactivate_gyroscope(self):
        if self.is_gyroscope_running:
            self.is_gyroscope_running = False
            if self.gyroscope.is_running:
                self.gyroscope.stop()
            if self.gyro_thread.is_alive():
                self.gyro_thread.join()
            print("LOG: Gyroscope deactivated.")
        else:
            print("LOG: Gyroscope not activated!")