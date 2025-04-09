import math
import numpy as np
from model.custom_types.index import GyroData, FootPositions
from utils.math_utils import turn_points_with_euler_radians
from utils.ConfigHelper import ConfigHelper
from utils.utils import get_np_array_from_foot_positions, get_foot_positions_from_np_array

robotdog_config = ConfigHelper.get_section("robotdog_parameters")
movement_config = ConfigHelper.get_section("movement_parameters")
UPPER_LEG_LENGTH = robotdog_config.getfloat("upper_leg_length", fallback=10.0)
LOWER_LEG_LENGTH = robotdog_config.getfloat("lower_leg_length", fallback=10.0)
BODY_LENGTH = robotdog_config.getfloat("body_length", fallback=21.0)
BODY_WIDTH = robotdog_config.getfloat("body_width", fallback=15.8)
MAX_HEIGHT = movement_config.getfloat("max_height", fallback=15.0)

SHOULDER_POSITIONS = np.asfortranarray([
    [BODY_LENGTH / 2, BODY_LENGTH / 2, -BODY_LENGTH / 2, -BODY_LENGTH / 2],
    [-BODY_WIDTH / 2, BODY_WIDTH / 2, -BODY_WIDTH / 2, BODY_WIDTH / 2],
    [0, 0, 0, 0],
])

def get_angle_from_position(
    x: float, 
    y: float, 
    z: float
) -> tuple[float, float, float]:
    return inverse_kinematics(x, y, z, UPPER_LEG_LENGTH, LOWER_LEG_LENGTH)

def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    a1: float = UPPER_LEG_LENGTH,
    a2: float = LOWER_LEG_LENGTH
) -> tuple[float, float, float]:
    """
    Computes joint angles from desired foot position using inverse kinematics.
    Returns (shoulder, elbow, hip) angles in degrees.
    """
    y_prime = -math.sqrt(z**2 + y**2)
    thetaz = math.atan2(abs(y), z)

    c2 = (x**2 + y_prime**2 - a1**2 - a2**2) / (2 * a1 * a2)
    s2 = math.sqrt(abs(1 - c2**2))
    theta2 = math.atan2(s2, c2)

    c1 = (x * (a1 + a2 * c2) + y_prime * (a2 * s2)) / (x**2 + y_prime**2)
    s1 = (y_prime * (a1 + a2 * c2) - x * (a2 * s2)) / (x**2 + y_prime**2)
    theta1 = math.atan2(s1, c1)

    theta_shoulder: float = -theta1
    theta_elbow: float = theta_shoulder - theta2
    theta_hip: float = thetaz

    return (
        math.degrees(theta_shoulder),
        math.degrees(theta_elbow),
        math.degrees(theta_hip),
    )
    

def forward_kinematics(
    theta_shoulder: float,
    theta_elbow: float,
    theta_hip: float,
    a1: float = UPPER_LEG_LENGTH,
    a2: float = LOWER_LEG_LENGTH
) -> tuple[float, float, float]:
    """
    Calculate (x, y, z) foot position from joint angles.
    Currently simplified to ignore hip (z-axis tilt).
    """
    theta_shoulder_rad = math.radians(theta_shoulder)
    theta_elbow_rad = math.radians(theta_elbow)

    x = a1 * math.cos(theta_shoulder_rad) + a2 * math.cos(theta_elbow_rad)
    y_prime = a1 * math.sin(theta_shoulder_rad) + a2 * math.sin(theta_elbow_rad)
    z = 0  # simplified, no hip tilt applied
    y = -math.sqrt(max(y_prime**2 - z**2, 0.0))

    return x, y, z


def compensate_foot_positions_by_gyro(foot_positions: FootPositions, gyro_data: GyroData) -> FootPositions:
    np_foot_positions = get_np_array_from_foot_positions(foot_positions, order='xzy')
    correction_factor = 0.8
    max_tilt = 0.4
    roll_compensation = correction_factor * np.clip(-gyro_data.roll, -max_tilt, max_tilt)
    pitch_compensation = correction_factor * np.clip(-gyro_data.pitch, -max_tilt, max_tilt)
    np_rotated_foot_positions = turn_points_with_euler_radians(np_foot_positions, roll_compensation, pitch_compensation, 0)
    rotated_foot_positions = get_foot_positions_from_np_array(np_rotated_foot_positions)
    return rotated_foot_positions