import time
import numpy as np
import logging
from threading import Thread, Event

from src.model.custom_types.index import LegPosition, BehaviorState, FootPositions, Position
from src.model.kinematics import get_angle_from_position, compensate_foot_positions_by_gyro
from src.model.MotionGenerator import MotionGenerator
from src.model.StateManager import StateManager
from src.model.LegController import LegController
from src.model.GyroController import GyroController
from src.model.Pose import Pose
from src.utils.ConfigHelper import ConfigHelper

logger = logging.getLogger(__name__)

class MovementExecutor:
    movement_config = ConfigHelper.get_section("movement_parameters")
    MAX_HEIGHT = movement_config.getfloat("max_height", fallback=15.0)
    MIN_HEIGHT = movement_config.getfloat("min_height", fallback=10.0)
    DEFAULT_STAND_FOOT_POSITIONS = FootPositions(
        FL=Position(x=0, y=-MAX_HEIGHT, z=0),
        FR=Position(x=0, y=-MAX_HEIGHT, z=0),
        BL=Position(x=0, y=-MAX_HEIGHT, z=0),
        BR=Position(x=0, y=-MAX_HEIGHT, z=0),
    )

    def __init__(self, state_manager: StateManager, leg_controllers: dict[LegPosition, LegController], gyro_controller: GyroController, gyro_event: Event):
        self.state_manager = state_manager
        self.legs = leg_controllers
        self.gyro_controller = gyro_controller
        self.gyro_event = gyro_event
        self.current_behavior = self.state_manager.get_behavior_state()

        self.moving_thread = Thread()

    def run(self):
        behavior = self.state_manager.get_behavior_state()

        if self.current_behavior != behavior:
            if self.moving_thread.is_alive():
                self.moving_thread.join()
            self.current_behavior = behavior

        if behavior == BehaviorState.STAND:
            logger.info("[MovementExecutor] Start standing...")
            self.moving_thread = Thread(target=self.standup, daemon=True)
            self.moving_thread.start()

        elif behavior == BehaviorState.MOVE:
            logger.info("[MovementExecutor] Start moving...")
            self.moving_thread = Thread(target=self.move, daemon=True)
            self.moving_thread.start()

        elif behavior == BehaviorState.CALIBRATE:
            logger.info("[MovementExecutor] Calibrating...")
            self.calibrate()

        elif behavior == BehaviorState.POSE:
            logger.info("[MovementExecutor] Applying pose...")
            self.moving_thread = Thread(target=self.apply_pose, daemon=True)
            self.moving_thread.start()


    def standup(self):
        if self.state_manager.get_foot_positions() is None:
            self.state_manager.set_foot_positions(self.DEFAULT_STAND_FOOT_POSITIONS)

        delay_time = self.state_manager.get_delay_time()
        last_loop = time.time()

        while self.state_manager.get_behavior_state() == BehaviorState.STAND:
            now = time.time()
            if now - last_loop < delay_time:
                continue
            last_loop = time.time()

            foot_current_positions = self.state_manager.get_foot_positions()

            if self.gyro_event.is_set():
                gyro_data = self.gyro_controller.read_gyro_data()
                if gyro_data:
                    foot_current_positions = compensate_foot_positions_by_gyro(foot_current_positions, gyro_data)

            self._set_motors_by_foot_positions(foot_current_positions)

    def move(self):
        if self.state_manager.get_foot_positions() is None:
            self.state_manager.set_foot_positions(self.DEFAULT_STAND_FOOT_POSITIONS)

        tick = 0
        delay_time = self.state_manager.get_delay_time()
        last_loop = time.time()

        while self.state_manager.get_behavior_state() == BehaviorState.MOVE:
            now = time.time()
            if now - last_loop < delay_time:
                continue
            last_loop = time.time()

            x_velocity, z_velocity = self.state_manager.get_horizontal_velocity()
            yaw_rate = self.state_manager.get_yaw_rate()
            y_height = self.state_manager.get_height()
            foot_current_positions = self.state_manager.get_foot_positions()

            logger.debug(f"[MovementExecutor.move] Current foot positions: {foot_current_positions}")
            logger.debug(f"[MovementExecutor.move] Velocities: x={x_velocity}, z={z_velocity}, yaw={yaw_rate}")
            logger.debug(f"[MovementExecutor.move] Height: {y_height}")

            if self.gyro_event.is_set():
                gyro_data = self.gyro_controller.read_gyro_data()
                if gyro_data:
                    foot_current_positions = compensate_foot_positions_by_gyro(foot_current_positions, gyro_data)
                    logger.debug(f"[MovementExecutor.move] Gyro compensated positions: {foot_current_positions}")

            # 每隻腳產生動作軌跡
            motions = {
                leg: MotionGenerator.generate_motion_from_position(foot_current_positions[leg]) *
                     np.array([x_velocity, z_velocity, y_height])[:, None]
                for leg in LegPosition
            }

            # 若有旋轉指令，套用轉彎修正
            if yaw_rate is not None and yaw_rate != 0.0:
                for leg in LegPosition:
                    motions[leg] = self._adjust_for_turning(motions[leg], yaw_rate, leg)
                logger.debug(f"[MovementExecutor.move] Adjusted motion by yaw_rate: {yaw_rate}")
            logger.debug(f"[MovementExecutor.move] Generated motions: {motions}")

            i1 = tick % 40
            i2 = (tick + 20) % 40

            X, Z, Y = 0, 1, 2
            foot_next_positions = FootPositions(
                FL=Position(motions[LegPosition.FL][X][i1] + 3, motions[LegPosition.FL][Y][i1], motions[LegPosition.FL][Z][i1]),
                FR=Position(motions[LegPosition.FR][X][i2] + 3, motions[LegPosition.FR][Y][i2], motions[LegPosition.FR][Z][i2]),
                BL=Position(motions[LegPosition.BL][X][i2], motions[LegPosition.BL][Y][i2], motions[LegPosition.BL][Z][i2]),
                BR=Position(motions[LegPosition.BR][X][i1], motions[LegPosition.BR][Y][i1], motions[LegPosition.BR][Z][i1]),
            )

            logger.debug(f"[MovementExecutor.move] Next foot positions: {foot_next_positions}")
            logger.debug(f"[MovementExecutor.move] Tick: {tick}")

            self._set_motors_by_foot_positions(foot_next_positions)
            tick += 1


    def calibrate(self):
        self._set_all_leg_angles(shoulder=180, elbow=40, hip=90)

    def _set_motors_by_foot_positions(self, foot_positions: FootPositions):
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

        self._set_leg_angle(leg_position=LegPosition.FL, shoulder=theta_shoulder_FL, elbow=theta_elbow_FL, hip=theta_hip_FL)
        self._set_leg_angle(leg_position=LegPosition.FR, shoulder=theta_shoulder_FR, elbow=theta_elbow_FR, hip=theta_hip_FR)
        self._set_leg_angle(leg_position=LegPosition.BL, shoulder=theta_shoulder_BL, elbow=theta_elbow_BL, hip=theta_hip_BL)
        self._set_leg_angle(leg_position=LegPosition.BR, shoulder=theta_shoulder_BR, elbow=theta_elbow_BR, hip=theta_hip_BR)
            
    def _set_leg_angle(self, leg_position: LegPosition, shoulder: float, elbow: float, hip: float):
        self.legs[leg_position].set_shoulder_angle(shoulder)
        self.legs[leg_position].set_elbow_angle(elbow)
        self.legs[leg_position].set_hip_angle(hip)

    def _set_all_leg_angles(self, shoulder: float, elbow: float, hip: float):
        for leg in LegPosition:
            self.legs[leg].set_shoulder_angle(shoulder)
            self.legs[leg].set_elbow_angle(elbow)
            self.legs[leg].set_hip_angle(hip)

    def _adjust_for_turning(self, motion: np.ndarray, turning_factor: float, leg: LegPosition) -> np.ndarray:
        if turning_factor > 0:  # Turn Right
            if leg in (LegPosition.FL, LegPosition.BL):
                motion[0, :] *= (1 + turning_factor)
            else:
                motion[0, :] *= (1 - turning_factor)
        elif turning_factor < 0:  # Turn Left
            turning_factor = abs(turning_factor)
            if leg in (LegPosition.FL, LegPosition.BL):
                motion[0, :] *= (1 - turning_factor)
            else:
                motion[0, :] *= (1 + turning_factor)
        return motion
    
    def calibrate_for_installation_1(self):
        self._set_all_leg_angles(90, 0, 90)

    def calibrate_for_installation_2(self):
        self._set_all_leg_angles(180, 90, 90)

    def apply_pose(self):
        pose_name = self.state_manager.get_pose()
        if not pose_name:
            logger.info("[MovementExecutor] No pose name specified")
            return
        pose_positions = Pose.get_pose_positions(pose_name)
        if pose_positions:
            self._set_motors_by_foot_positions(pose_positions)
        else:
            logger.info(f"[MovementExecutor] Unknown pose: {pose_name}")
