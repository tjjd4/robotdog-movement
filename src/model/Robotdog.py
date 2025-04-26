from threading import Thread, Event
from typing import Optional

from .custom_types.index import LegPosition, RobotDogState, MotionCommand, FootPositions, Position
from .hardware.Motor import Motor
from .LidarController import LidarController
from .LegController import LegController
from .GyroscopeController import GyroscopeController
from .CameraController import CameraController
from .MovementExecutor import MovementExecutor
from .StateManager import StateManager

from src.utils.ConfigHelper import ConfigHelper

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
        init_state = RobotDogState()
        init_state.delay_time = self.movement_config.getfloat("delay_time", fallback=0.01)
    
        self.state_manager = StateManager(init_state)
        self.moving_thread = Thread()
        self.standing_thread = Thread()
        self.gyro_event = Event()

        self.gyroscope = GyroscopeController()
        self.camera_controller = CameraController()
        self.lidar_controller = LidarController()
        self.movement_executor = MovementExecutor(self.state_manager, self.legs, self.gyroscope, self.gyro_event)
        
    def run(self, command: Optional[MotionCommand] = None):
        if command is None:
            return
        self.state_manager.update_state_by_motion_command(command)
        self.movement_executor.run()


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

    def get_camera_stream(self):
        return self.camera_controller.generate_frames()

    def calibrate(self):
        self.movement_executor.calibrate()

    def start_lidar(self):
        self.lidar_controller.start()

    def stop_lidar(self):
        self.lidar_controller.stop()

    def get_latest_scan(self):
        return self.lidar_controller.get_latest_scan()
    
    def get_lidar_stream(self):
        return self.lidar_controller.generate_frames()
    
    def calibrate_for_installation_1(self):
        self.movement_executor.calibrate_for_installation_1()

    def calibrate_for_installation_2(self):
        self.movement_executor.calibrate_for_installation_2()
