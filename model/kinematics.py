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

def get_angle_from_position(x: float, y: float, z: float):
    return inverse_kinematics(x,y,z,upper_leg_length,lower_leg_length)

def inverse_kinematics(x: float, y: float, z: float, a1: float=upper_leg_length, a2: float=lower_leg_length):
    y_prime = -math.sqrt((z)**2 + y**2)
    thetaz = math.atan2(abs(y), z)

    c2 = (x**2 + y_prime**2 - a1**2 - a2**2) / (2 * a1 * a2)
    s2 = math.sqrt(abs(1 - c2**2))
    theta2 = math.atan2(s2, c2)

    c1 = (x * (a1 + (a2 * c2)) + y_prime * (a2 * s2)) / (x**2 + y_prime**2)
    s1 = (y_prime * (a1 + (a2 * c2)) - x * (a2 * s2)) / (x**2 + y_prime**2)
    theta1 = math.atan2(s1, c1)

    theta_shoulder = -theta1
    theta_elbow = theta_shoulder - theta2
    theta_hip: float = thetaz

    theta_shoulder = math.degrees(theta_shoulder)
    theta_elbow = math.degrees(theta_elbow)
    theta_hip: float = math.degrees(theta_hip)

    return theta_shoulder, theta_elbow, theta_hip


def forward_kinematics(theta_shoulder: float, theta_elbow: float, theta_hip: float, a1: float, a2: float):
    # Convert angles from degrees to radians
    theta_shoulder_rad = math.radians(theta_shoulder)
    theta_elbow_rad = math.radians(theta_elbow)

    # Calculate x and y_prime (projection of y and z on 2D plane)
    x = a1 * math.cos(theta_shoulder_rad) + a2 * math.cos(theta_elbow_rad)
    y_prime = a1 * math.sin(theta_shoulder_rad) + a2 * math.sin(theta_elbow_rad)

    # Recover y and z from y_prime
    z = 0  # Simplified to assume L = 0, so z = 0

    y = -math.sqrt(y_prime**2 - (z)**2)  # y_prime = -sqrt(z^2 + y^2), assume z = 0

    return x, y, z


def compensate_foot_positions_by_gyro(foot_positions: FootPositions, gyro_data: GyroData) -> FootPositions:
    np_foot_positions = get_np_array_from_foot_positions(foot_positions, order='xzy')

    # Calculate body normal vector from gyro data
    roll_rad = math.radians(gyro_data.roll)
    pitch_rad = math.radians(gyro_data.pitch)
    body_normal = np.array([
        math.sin(roll_rad),
        -math.sin(pitch_rad) * math.cos(roll_rad),
        math.cos(pitch_rad) * math.cos(roll_rad)
    ])

    # Calculate compensation factor based on gyro data
    base_correction = 0.8
    severity = max(
        min(abs(gyro_data.roll) / 45.0, 1.0),
        min(abs(gyro_data.pitch) / 45.0, 1.0)
    )
    correction_factor = base_correction * (1.0 + severity)

    # Calculate maximum compensation and minimum leg height
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

    return get_foot_positions_from_np_array(compensated_positions)
