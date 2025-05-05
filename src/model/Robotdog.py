import logging
from threading import Event
from typing import Optional

from .custom_types.index import LegPosition, RobotDogState, MotionCommand
from .hardware.Motor import Motor
from .LidarController import LidarController
from .LegController import LegController
from .GyroController import GyroController
from .CameraController import CameraController
from .MovementExecutor import MovementExecutor
from .StateManager import StateManager

from src.utils.ConfigHelper import ConfigHelper

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Robotdog:
    robotdog_config = ConfigHelper.get_section("robotdog_parameters")
    movement_config = ConfigHelper.get_section("movement_parameters")
    legs_config = ConfigHelper.get_section("motors_legs")

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
        init_state.max_height = self.movement_config.getfloat("max_height", fallback=15.0)
    
        self.state_manager = StateManager(init_state)
        self.gyro_event = Event()

        self.gyro_controller = GyroController()
        self.camera_controller = CameraController()
        self.lidar_controller = LidarController()
        self.movement_executor = MovementExecutor(self.state_manager, self.legs, self.gyro_controller, self.gyro_event)
        
    def run(self, command: Optional[MotionCommand] = None):
        if command is None:
            return
        self.state_manager.update_state_by_motion_command(command)
        self.movement_executor.run()


    def activate_gyroscope(self):
        if not self.gyro_event.is_set():
            self.gyro_event.set()
            logger.info("[Robotdog] Gyroscope activated.")
        else:
            logger.info("[Robotdog] Gyroscope already activated!")

    def deactivate_gyroscope(self):
        if self.gyro_event.is_set():
            self.gyro_event.clear()
            logger.info("[Robotdog] Gyroscope deactivated.")
        else:
            logger.info("[Robotdog] Gyroscope not activated!")

    def start_camera(self):
        """Start the camera."""
        self.camera_controller.start_camera()
        logger.info("[Robotdog] Camera started.")

    def stop_camera(self):
        """Stop the camera."""
        self.camera_controller.stop_camera()
        logger.info("[Robotdog] Camera stopped.")

    def start_detection(self):
        """Start the object detection feature."""
        self.camera_controller.start_detection()
        logger.info("[Robotdog] Object detection started.")

    def stop_detection(self):
        """Stop the object detection feature."""
        self.camera_controller.stop_detection()
        logger.info("[Robotdog] Object detection stopped.")

    def get_camera_stream(self):
        return self.camera_controller.generate_frames()

    def calibrate(self):
        self.movement_executor.calibrate()

    def start_lidar(self):
        self.lidar_controller.start()
        logger.info("[Robotdog] Lidar started.")

    def stop_lidar(self):
        self.lidar_controller.stop()
        logger.info("[Robotdog] Lidar stopped.")

    def get_latest_scan(self):
        return self.lidar_controller.get_latest_scan()
    
    def get_lidar_stream(self):
        return self.lidar_controller.generate_frames()
    
    def calibrate_for_installation_1(self):
        self.movement_executor.calibrate_for_installation_1()
        logger.info("[Robotdog] Calibrated for installation 1.")

    def calibrate_for_installation_2(self):
        self.movement_executor.calibrate_for_installation_2()
        logger.info("[Robotdog] Calibrated for installation 2.")
