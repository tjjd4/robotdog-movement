import math
import numpy as np
from model.custom_types.index import GyroData, FootPositions
from utils.math_utils import turn_points_with_euler_radians
from utils.ConfigHelper import ConfigHelper
from utils.utils import get_np_array_from_foot_positions, get_foot_positions_from_np_array

robotdog_config = ConfigHelper.get_section("robotdog_parameters")
movement_config = ConfigHelper.get_section("movement_parameters")

upper_leg_length = robotdog_config.getfloat("upper_leg_length", fallback=10.0)
lower_leg_length = robotdog_config.getfloat("lower_leg_length", fallback=10.0)
body_length = robotdog_config.getfloat("body_length", fallback=21.0)
body_width = robotdog_config.getfloat("body_width", fallback=15.8)
max_height = movement_config.getfloat("max_height", fallback=15.0)

shoulder_positions = np.asfortranarray([
    [body_length / 2, body_length / 2, -body_length / 2, -body_length / 2],
    [-body_width / 2, body_width / 2, -body_width / 2, body_width / 2],
    [0, 0, 0, 0],
])

class GyroCompensator:
    def __init__(self, alpha: float = 0.3):
        self.previous_positions = None
        self.alpha = alpha

    def compensate(self, foot_positions: FootPositions, gyro_data: GyroData) -> FootPositions:
        np_foot_positions = get_np_array_from_foot_positions(foot_positions, order='xzy')

        # --- Roll & Pitch Compensation ---
        roll_rad = math.radians(gyro_data.roll)
        pitch_rad = math.radians(gyro_data.pitch)

        body_normal = np.array([
            math.sin(roll_rad),
            -math.sin(pitch_rad) * math.cos(roll_rad),
            math.cos(pitch_rad) * math.cos(roll_rad)
        ])

        base_correction = 0.8
        severity = max(
            min(abs(gyro_data.roll) / 45.0, 1.0),
            min(abs(gyro_data.pitch) / 45.0, 1.0)
        )
        correction_factor = base_correction * (1.0 + severity)

        max_compensation = 3.0
        max_leg_extension_y = -max_height * 1.2
        min_leg_extension_y = -max_height * 0.5

        compensated_positions = np_foot_positions.copy()

        for i in range(4):
            dx = shoulder_positions[0, i]
            dy = shoulder_positions[1, i]
            height_adj = (dx * body_normal[0] + dy * body_normal[1]) * correction_factor
            height_adj = np.clip(height_adj, -max_compensation, max_compensation)

            new_height = compensated_positions[2, i] - height_adj
            if new_height < max_leg_extension_y:
                height_adj = compensated_positions[2, i] - max_leg_extension_y
            elif new_height > min_leg_extension_y:
                height_adj = compensated_positions[2, i] - min_leg_extension_y

            compensated_positions[2, i] -= height_adj

        # --- Yaw Compensation ---
        yaw_rad = math.radians(gyro_data.yaw)
        rotation_matrix = np.array([
            [math.cos(yaw_rad), -math.sin(yaw_rad)],
            [math.sin(yaw_rad),  math.cos(yaw_rad)]
        ])
        compensated_positions[0:2, :] = rotation_matrix @ compensated_positions[0:2, :]

        # --- Smoothing ---
        if self.previous_positions is None:
            self.previous_positions = compensated_positions.copy()

        smoothed = self.alpha * compensated_positions + (1 - self.alpha) * self.previous_positions
        self.previous_positions = smoothed.copy()

        return get_foot_positions_from_np_array(smoothed)