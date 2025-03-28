import time
from threading import Thread, Event
from typing import Optional
import numpy as np

from .custom_types.index import LegPosition, LegPart, RobotDogState, BehaviorState, MotionCommand, FootPositions, Position
from .hardware.Motor import Motor
from .LegController import LegController
from .GyroscopeController import GyroscopeController
from .MotionGenerator import MotionGenerator
from .CameraController import CameraController
from .kinematics import get_angle_from_position, compensate_foot_positions_by_gyro

from utils.ConfigHelper import ConfigHelper

class Robotdog:
    robotdog_config = ConfigHelper.get_section("robotdog_parameters")
    movement_config = ConfigHelper.get_section("movement_parameters")
    legs_config = ConfigHelper.get_section("motors_legs")
    MAX_HEIGHT = movement_config.getfloat("max_height", fallback=15.0)
    DEFAULT_STAND_FOOT_POSITIONS = FootPositions(
        FL = Position(x=0, y=-MAX_HEIGHT, z=0),
        FR = Position(x=0, y=-MAX_HEIGHT, z=0),
        BL = Position(x=0, y=-MAX_HEIGHT, z=0),
        BR = Position(x=0, y=-MAX_HEIGHT, z=0)
    )

    def __init__(self) -> None:
        self.upper_leg_length = self.robotdog_config.getfloat("upper_leg_length", fallback=10.0)
        self.lower_leg_length = self.robotdog_config.getfloat("lower_leg_length", fallback=10.0)
        self.delay_time = self.movement_config.getfloat("delay_time", fallback=0.01)
        self.legs: dict[LegPosition, LegController] = {
            LegPosition.FL: LegController(
                Motor.FL_SHOULDER,
                Motor.FL_ELBOW, Motor.FL_HIP,
                FB_is_opposited=self.legs_config.getboolean("FB_FL_is_opposited", fallback=False),
                LR_is_opposited=self.legs_config.getboolean("LR_FL_is_opposited", fallback=False),
            ),
            LegPosition.FR: LegController(
                Motor.FR_SHOULDER,
                Motor.FR_ELBOW,
                Motor.FR_HIP,
                FB_is_opposited=self.legs_config.getboolean("FB_FR_is_opposited", fallback=True),
                LR_is_opposited=self.legs_config.getboolean("LR_FR_is_opposited", fallback=False),
            ),
            LegPosition.BL: LegController(
                Motor.BL_SHOULDER,
                Motor.BL_ELBOW, Motor.BL_HIP,
                FB_is_opposited=self.legs_config.getboolean("FB_BL_is_opposited", fallback=False),
                LR_is_opposited=self.legs_config.getboolean("LR_BL_is_opposited", fallback=True),
            ),
            LegPosition.BR: LegController(
                Motor.BR_SHOULDER,
                Motor.BR_ELBOW,
                Motor.BR_HIP,
                FB_is_opposited=self.legs_config.getboolean("FB_BR_is_opposited", fallback=True),
                LR_is_opposited=self.legs_config.getboolean("LR_BR_is_opposited", fallback=True),
            ),
        }
        self.state = RobotDogState()
        self.moving_thread = Thread()
        self.standing_thread = Thread()
        self.gyroscope = GyroscopeController()
        self.gyro_event = Event()

        self.camera_controller = CameraController()

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


    def adjust_for_turning(self, motion: np.ndarray, turning_factor: float, leg_position: LegPosition) -> np.ndarray:
        """
        Adjust the step size for turning using the small-step, big-step method.

        Args:
            motion (np.ndarray): Original motion trajectory (shape: [3, n_steps]).
            turning_factor (float): Positive for turning right, negative for turning left.

        Returns:
            np.ndarray: Adjusted motion trajectory.
        """

        if turning_factor > 0:  # Turn Right
            # Left legs take bigger steps, right legs take smaller steps
            if leg_position == LegPosition.FL or leg_position == LegPosition.BL:
                motion[0, :] *= (1 + turning_factor)  # Left legs bigger steps
            elif leg_position == LegPosition.FR or leg_position == LegPosition.BR:
                motion[0, :] *= (1 - turning_factor)  # Right legs smaller steps
        elif turning_factor < 0:  # Turn Left
            turning_factor = abs(turning_factor)
            # Right legs take bigger steps, left legs take smaller steps
            if leg_position == LegPosition.FL or leg_position == LegPosition.BL:
                motion[0, :] *= (1 - turning_factor)  # Left legs smaller steps
            elif leg_position == LegPosition.FR or leg_position == LegPosition.BR:
                motion[0, :] *= (1 + turning_factor)  # Right legs bigger steps

        return motion

    def set_motors_by_foot_positions(self, foot_positions: FootPositions):

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
        if self.state.foot_current_positions == None:
            self.state.foot_current_positions = self.DEFAULT_STAND_FOOT_POSITIONS
        last_loop = time.time()
        while self.state.behavior_state == BehaviorState.STAND:
            now = time.time()
            if now - last_loop < self.delay_time:
                continue
            last_loop = time.time()
            foot_current_positions = self.state.foot_current_positions

            if self.gyro_event.is_set():
                gyro_data = self.gyroscope.read_gyro_data()
                print(gyro_data)
                if gyro_data:
                    foot_current_positions = compensate_foot_positions_by_gyro(foot_current_positions, gyro_data)

            self.set_motors_by_foot_positions(foot_positions=foot_current_positions)


    def move(self):
        if self.state.foot_current_positions == None:
            self.state.foot_current_positions = self.DEFAULT_STAND_FOOT_POSITIONS
        tick = 0

        # Generate footstep
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
            foot_current_positions = self.state.foot_current_positions
            if self.gyro_event.is_set():
                gyro_data = self.gyroscope.read_gyro_data()
                if gyro_data:
                    foot_current_positions = compensate_foot_positions_by_gyro(foot_current_positions, gyro_data)

            four_leg_motions = np.asfortranarray([
                MotionGenerator.generate_motion_from_position(foot_current_positions.FL) * np.array([x_velocity, z_velocity, y_height])[:, None],
                MotionGenerator.generate_motion_from_position(foot_current_positions.FR) * np.array([x_velocity, z_velocity, y_height])[:, None],
                MotionGenerator.generate_motion_from_position(foot_current_positions.BL) * np.array([x_velocity, z_velocity, y_height])[:, None],
                MotionGenerator.generate_motion_from_position(foot_current_positions.BR) * np.array([x_velocity, z_velocity, y_height])[:, None]
            ])

            if yaw_rate != 0.0:
                for leg_position in LegPosition:
                    four_leg_motions[leg_position] = self.adjust_for_turning(four_leg_motions[leg_position], self.state.yaw_rate, leg_position)

            # 在 four_leg_motions 中 x, z, y 座標的 index
            X, Z, Y = 0, 1, 2

            i1 = tick % 40
            i2 = (tick + 20) % 40

            foot_next_positions = FootPositions(
                FL = Position(x=four_leg_motions[LegPosition.FL][X][i1]+3, y=four_leg_motions[LegPosition.FL][Y][i1], z=four_leg_motions[LegPosition.FL][Z][i1]),
                FR = Position(x=four_leg_motions[LegPosition.FR][X][i2]+3, y=four_leg_motions[LegPosition.FR][Y][i2], z=four_leg_motions[LegPosition.FR][Z][i2]),
                BL = Position(x=four_leg_motions[LegPosition.BL][X][i2], y=four_leg_motions[LegPosition.BL][Y][i2], z=four_leg_motions[LegPosition.BL][Z][i2]),
                BR = Position(x=four_leg_motions[LegPosition.BR][X][i1], y=four_leg_motions[LegPosition.BR][Y][i1], z=four_leg_motions[LegPosition.BR][Z][i1])
            )
            self.set_motors_by_foot_positions(foot_positions=foot_next_positions)

            tick += 1

            if np.allclose(self.state.horizontal_velocity, [0, 0]):
                print("Stopping robot...")
                self.state.behavior_state = BehaviorState.REST


    def run(self, command: Optional[MotionCommand]=None):
        """主狀態機控制流程"""
        if command == None:
            return
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

    def activate_gyroscope(self):
        if not self.gyro_event.is_set():
            self.gyro_event.set()
            print("LOG: Gyroscope activated.")
        else:
            print("LOG: Gyroscope already activated!")

    def deactivate_gyroscope(self):
        if self.gyro_event.is_set():
            self.gyro_event.clear()
            print("LOG: Gyroscope deactivated.")
        else:
            print("LOG: Gyroscope not activated!")

    def start_camera(self):
        """Start the camera."""
        self.camera_controller.start_camera()
        print("Camera started.")

    def stop_camera(self):
        """Stop the camera."""
        self.camera_controller.stop_camera()
        print("Camera stopped.")

    def start_detection(self):
        """Start the object detection feature."""
        self.camera_controller.start_detection()
        print("Object detection started.")

    def stop_detection(self):
        """Stop the object detection feature."""
        self.camera_controller.stop_detection()
        print("Object detection stopped.")

    def start_camera_server(self):
        """Start the camera server to view the live feed."""
        self.camera_controller.start_server()
        print("Camera server started. View at http://localhost:5000/video")
