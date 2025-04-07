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
from .MovementExecutor import MovementExecutor
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
        # self.upper_leg_length = self.robotdog_config.getfloat("upper_leg_length", fallback=10.0)
        # self.lower_leg_length = self.robotdog_config.getfloat("lower_leg_length", fallback=10.0)
        self.legs: dict[LegPosition, LegController] = {
            LegPosition.FL: LegController(Motor.FL_SHOULDER, Motor.FL_ELBOW, Motor.FL_HIP,
                FB_is_opposited=self.legs_config.getboolean("FB_FL_is_opposited", fallback=False),
                LR_is_opposited=self.legs_config.getboolean("LR_FL_is_opposited", fallback=False),
            ),
            LegPosition.FR: LegController(Motor.FR_SHOULDER, Motor.FR_ELBOW, Motor.FR_HIP,
                FB_is_opposited=self.legs_config.getboolean("FB_FR_is_opposited", fallback=True),
                LR_is_opposited=self.legs_config.getboolean("LR_FR_is_opposited", fallback=False),
            ),
            LegPosition.BL: LegController(Motor.BL_SHOULDER, Motor.BL_ELBOW, Motor.BL_HIP,
                FB_is_opposited=self.legs_config.getboolean("FB_BL_is_opposited", fallback=False),
                LR_is_opposited=self.legs_config.getboolean("LR_BL_is_opposited", fallback=True),
            ),
            LegPosition.BR: LegController(Motor.BR_SHOULDER, Motor.BR_ELBOW, Motor.BR_HIP,
                FB_is_opposited=self.legs_config.getboolean("FB_BR_is_opposited", fallback=True),
                LR_is_opposited=self.legs_config.getboolean("LR_BR_is_opposited", fallback=True),
            ),
        }
        self.state = RobotDogState()
        self.state.delay_time = self.movement_config.getfloat("delay_time", fallback=0.01)
        self.moving_thread = Thread()
        self.standing_thread = Thread()
        self.gyro_event = Event()

        self.gyroscope = GyroscopeController()
        # self.camera_controller = CameraController()
        self.movement_executor = MovementExecutor(self.state, self.legs, self.gyroscope, self.gyro_event)
        
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
                self.standing_thread = Thread(target=self.movement_executor.standup, daemon=True)
                self.standing_thread.start()

        elif self.state.behavior_state == BehaviorState.MOVE:
            if self.standing_thread.is_alive():
                self.standing_thread.join()

            if not self.moving_thread.is_alive():
                print("Start moving thread...")
                self.moving_thread = Thread(target=self.movement_executor.move, daemon=True)
                self.moving_thread.start()

        elif self.state.behavior_state == BehaviorState.CALIBRATE:
            if self.moving_thread.is_alive():
                self.moving_thread.join()
            if self.standing_thread.is_alive():
                self.standing_thread.join()
            self.movement_executor.calibrate()

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

    def calibrate(self):
        self.movement_executor.calibrate()