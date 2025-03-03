import time
from threading import Thread
import numpy as np
import queue

from .types.types import LegPosition, LegPart, RobotDogState, BehaviorState, MotionCommand, FootPositions, Position, GyroData
from .hardware.Motor import Motor
from .LegController import LegController
from .GyroscopeController import GyroscopeController
from .MotionGenerator import MotionGenerator
from .kinematics import get_angle_from_position, compensate_foot_positions_by_gyro

from utils.ConfigHelper import ConfigHelper
from utils.GyroQueue import GyroQueue

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
        self.gyro_queue = GyroQueue(maxsize=1)
        self.gyroscope = GyroscopeController(gyro_queue=self.gyro_queue)
        self.state = RobotDogState()
        self.moving_thread = Thread()
        self.standing_thread = Thread()
        self.gyro_thread = Thread()

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
            self.set_leg_angle(leg_position, shoulder_angle, elbow_angle, hip_angle)


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

    def set_motors_by_foot_positions(self, foot_positions: FootPositions, gyro_data: GyroData=None):
        if self.state.is_gyro_running and gyro_data != None:
            foot_positions = compensate_foot_positions_by_gyro(foot_positions, gyro_data)

        self.update_foot_positions(foot_positions)

        theta_shoulder_FL, theta_elbow_FL, theta_hip_FL = get_angle_from_position(
            x=foot_positions.FL.x, y=foot_positions.FL.y, z=foot_positions.FL.z
        )
        theta_shoulder_FR, theta_elbow_FR, theta_hip_FR = get_angle_from_position(
            x=foot_positions.FR.x, y=foot_positions.FR.y, z=foot_positions.FR.z
        )
        theta_shoulder_BL, theta_elbow_BL, theta_hip_BL = get_angle_from_position(
            x=foot_positions.BL.x, y=foot_positions.BL.y, z=foot_positions.BL.z
        )
        theta_shoulder_BR, theta_elbow_BR, theta_hip_BR = get_angle_from_position(
            x=foot_positions.BR.x, y=foot_positions.BR.y, z=foot_positions.BR.z
        )

        self.set_leg_angle(LegPosition.FL, theta_shoulder_FL, theta_elbow_FL, theta_hip_FL)
        self.set_leg_angle(LegPosition.FR, theta_shoulder_FR, theta_elbow_FR, theta_hip_FR)
        self.set_leg_angle(LegPosition.BL, theta_shoulder_BL, theta_elbow_BL, theta_hip_BL)
        self.set_leg_angle(LegPosition.BR, theta_shoulder_BR, theta_elbow_BR, theta_hip_BR)

    """
    Movement Functions
    """

    def calibrate(self):
        self.set_four_legs_angle(180, 30, 90)

    def standup(self):
        last_loop = time.time()
        while self.state.behavior_state == BehaviorState.STAND:
            now = time.time()
            if now - last_loop < self.delay_time:
                continue
            last_loop = time.time()
            x, y, z = (0, -15, 0)

            gyro_data = None
            if self.state.is_gyro_running:
                gyro_data = self.state.gyro_data

            foot_new_positions = FootPositions(
                FL = Position(x=x, y=y, z=z),
                FR = Position(x=x, y=y, z=z),
                BL = Position(x=x, y=y, z=z),
                BR = Position(x=x, y=y, z=z)
            )

            self.set_motors_by_foot_positions(foot_positions=foot_new_positions, gyro_data=gyro_data)


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
            if self.state.is_gyro_running:
                gyro_data = self.state.gyro_data

            foot_new_positions = FootPositions(
                FL = Position(x=left_x[i1]+3, y=left_y[i1], z=left_z[i1]),
                FR = Position(x=right_x[i2]+3, y=right_y[i2], z=right_z[i2]),
                BL = Position(x=left_x[i2], y=left_y[i2], z=left_z[i2]),
                BR = Position(x=right_x[i1], y=right_y[i1], z=right_z[i1])
            )
            self.set_motors_by_foot_positions(foot_positions=foot_new_positions, gyro_data=gyro_data)

            index += 1

            if np.allclose(self.state.horizontal_velocity, [0, 0]):
                print("Stopping robot...")
                self.state.behavior_state = BehaviorState.REST


    def run(self, command: MotionCommand=None):
        """主狀態機控制流程"""
        if command:
            self.update_state_by_motion_command(command)

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

    def update_state_by_motion_command(self, command: MotionCommand):
        """根據指令更新狀態"""
        self.state.horizontal_velocity = command.horizontal_velocity
        self.state.yaw_rate = command.yaw_rate
        self.state.roll = command.roll
        self.state.pitch = command.pitch
        self.state.yaw = command.yaw
        self.state.height = command.height
        self.state.behavior_state = command.behavior_state

    def update_foot_positions(self, foot_current_positions: FootPositions):
        self.state.foot_current_positions = foot_current_positions

    def update_gyro_data(self):
        while self.state.is_gyro_running:
            try:
            # 直接拿最新的一筆資料 (LIFO)
                latest_gyro_data = self.gyro_queue.get_nowait()  # 不會阻塞
                with self.gyro_queue.mutex and self.gyro_queue.qsize() > 2:
                    self.gyro_queue.queue.clear()
                # 更新狀態
                self.state.gyro_data = latest_gyro_data
            except queue.Empty:
                pass

            time.sleep(0.25)

    def activate_gyroscope(self):
        if not self.state.is_gyro_running:
            if not self.gyroscope.is_running():
                self.gyroscope.start()
            if not self.gyro_thread.is_alive():
                self.gyro_thread = Thread(target=self.update_gyro_data, daemon=True)
                self.gyro_thread.start()

            self.state.is_gyro_running = True
            print("LOG: Gyroscope activated.")
        else:
            print("LOG: Gyroscope already activated!")

    def deactivate_gyroscope(self):
        if self.state.is_gyro_running:
            if self.gyroscope.is_running:
                self.gyroscope.stop()
            if self.gyro_thread.is_alive():
                self.gyro_thread.join()

            self.state.is_gyro_running = False
            print("LOG: Gyroscope deactivated.")
        else:
            print("LOG: Gyroscope not activated!")