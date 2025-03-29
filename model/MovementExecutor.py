import time
import numpy as np
from threading import Event

from custom_types.index import LegPosition, BehaviorState, FootPositions, Position
from motion.Kinematics import get_angle_from_position, compensate_foot_positions_by_gyro
from motion.MotionGenerator import MotionGenerator


class MovementExecutor:
    def __init__(self, robot_state, leg_controllers, gyroscope, gyro_event: Event):
        self.state = robot_state
        self.legs = leg_controllers
        self.gyroscope = gyroscope
        self.gyro_event = gyro_event

    def standup(self):
        if self.state.foot_current_positions is None:
            self.state.foot_current_positions = self._default_stand_foot_positions()

        delay_time = self.state.delay_time
        last_loop = time.time()

        while self.state.behavior_state == BehaviorState.STAND:
            now = time.time()
            if now - last_loop < delay_time:
                continue
            last_loop = time.time()

            foot_current_positions = self.state.foot_current_positions

            if self.gyro_event.is_set():
                gyro_data = self.gyroscope.read_gyro_data()
                if gyro_data:
                    foot_current_positions = compensate_foot_positions_by_gyro(foot_current_positions, gyro_data)

            self._set_motors_by_foot_positions(foot_current_positions)

    def move(self):
        if self.state.foot_current_positions is None:
            self.state.foot_current_positions = self._default_stand_foot_positions()

        tick = 0
        delay_time = self.state.delay_time
        last_loop = time.time()

        while self.state.behavior_state == BehaviorState.MOVE:
            now = time.time()
            if now - last_loop < delay_time:
                continue
            last_loop = time.time()

            x_velocity, z_velocity = self.state.horizontal_velocity
            yaw_rate = self.state.yaw_rate
            y_height = self.state.height
            foot_current_positions = self.state.foot_current_positions

            if self.gyro_event.is_set():
                gyro_data = self.gyroscope.read_gyro_data()
                if gyro_data:
                    foot_current_positions = compensate_foot_positions_by_gyro(foot_current_positions, gyro_data)

            # 每隻腳產生動作軌跡
            motions = {
                leg: MotionGenerator.generate_motion_from_position(foot_current_positions[leg]) *
                     np.array([x_velocity, z_velocity, y_height])[:, None]
                for leg in LegPosition
            }

            # 若有旋轉指令，套用轉彎修正
            if yaw_rate != 0.0:
                for leg in LegPosition:
                    motions[leg] = self._adjust_for_turning(motions[leg], yaw_rate, leg)

            i1 = tick % 40
            i2 = (tick + 20) % 40

            X, Z, Y = 0, 1, 2
            foot_next_positions = FootPositions(
                FL=Position(motions[LegPosition.FL][X][i1] + 3, motions[LegPosition.FL][Y][i1], motions[LegPosition.FL][Z][i1]),
                FR=Position(motions[LegPosition.FR][X][i2] + 3, motions[LegPosition.FR][Y][i2], motions[LegPosition.FR][Z][i2]),
                BL=Position(motions[LegPosition.BL][X][i2], motions[LegPosition.BL][Y][i2], motions[LegPosition.BL][Z][i2]),
                BR=Position(motions[LegPosition.BR][X][i1], motions[LegPosition.BR][Y][i1], motions[LegPosition.BR][Z][i1]),
            )

            self._set_motors_by_foot_positions(foot_next_positions)
            tick += 1

            if np.allclose(self.state.horizontal_velocity, [0, 0]):
                print("Stopping robot...")
                self.state.behavior_state = BehaviorState.REST

    def calibrate(self):
        self._set_all_leg_angles(shoulder=180, elbow=30, hip=90)

    def _set_motors_by_foot_positions(self, foot_positions: FootPositions):
        for leg in LegPosition:
            x, y, z = foot_positions[leg].x, foot_positions[leg].y, foot_positions[leg].z
            shoulder, elbow, hip = get_angle_from_position(x, y, z)
            self.legs[leg].set_shoulder_angle(shoulder)
            self.legs[leg].set_elbow_angle(elbow)
            self.legs[leg].set_hip_angle(hip)

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

    def _default_stand_foot_positions(self) -> FootPositions:
        max_height = self.state.height or 15.0
        return FootPositions(
            FL=Position(x=0, y=-max_height, z=0),
            FR=Position(x=0, y=-max_height, z=0),
            BL=Position(x=0, y=-max_height, z=0),
            BR=Position(x=0, y=-max_height, z=0),
        )
