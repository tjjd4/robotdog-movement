from model.kinematics import inverse_kinematics, forward_kinematics, compensate_foot_positions_by_gyro
from model.types.types import FootPositions, Position, GyroData
from utils.utils import get_np_array_from_foot_positions

if __name__ == '__main__':
    upper_leg = 10
    lower_leg = 10
    x, y, z = 0, -15, 0
    theta_shoulder, theta_elbow, theta_hip = inverse_kinematics(x, y, z, upper_leg, lower_leg)

    print(f"Position: {x}, {y}, {z}")
    print(f"Theta Shoulder: {theta_shoulder}")
    print(f"Theta Elbow: {theta_elbow}")
    print(f"Theta Hip: {theta_hip}")

    x1, y1, z1 = forward_kinematics(theta_shoulder, theta_elbow, theta_hip, upper_leg, lower_leg)
    print(f"Forward (x, y, z): {x1}, {y1}, {z1}")

    foot_current_positions = FootPositions(
        FL = Position(x=x, y=y, z=z),
        FR = Position(x=x, y=y, z=z),
        BL = Position(x=x, y=y, z=z),
        BR = Position(x=x, y=y, z=z)
    )
    print(get_np_array_from_foot_positions(foot_current_positions))

    gyro_data = GyroData(roll=10, pitch=10, yaw=0)

    compensated_foot_positions = compensate_foot_positions_by_gyro(foot_positions=foot_current_positions, gyro_data=gyro_data)

    print(get_np_array_from_foot_positions(compensated_foot_positions))