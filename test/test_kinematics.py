from model.kinematics import inverse_kinematics, forward_kinematics, compensate_legs_positions_by_gyro
from model.types.types import LegsPositions, Position, GyroData
from utils.utils import get_np_array_from_legs_positions

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

    legs_current_positions = LegsPositions(
        FL = Position(x=x, y=y, z=z),
        FR = Position(x=x, y=y, z=z),
        BL = Position(x=x, y=y, z=z),
        BR = Position(x=x, y=y, z=z)
    )
    print(get_np_array_from_legs_positions(legs_current_positions))

    gyro_data = GyroData(roll=10, pitch=10, yaw=0)

    compensated_legs_positions = compensate_legs_positions_by_gyro(legs_positions=legs_current_positions, gyro_data=gyro_data)

    print(get_np_array_from_legs_positions(compensated_legs_positions))