import sys
import os

move_logic_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(move_logic_path)

from model.kinematics import inverse_kinematics, forward_kinematics

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

    x, y, z = 0, -10, 0
    theta_shoulder, theta_elbow, theta_hip = inverse_kinematics(x, y, z, upper_leg, lower_leg)

    print(f"Position: {x}, {y}, {z}")
    print(f"Theta Shoulder: {theta_shoulder}")
    print(f"Theta Elbow: {theta_elbow}")
    print(f"Theta Hip: {theta_hip}")

    x2, y2, z2 = forward_kinematics(theta_shoulder, theta_elbow, theta_hip, upper_leg, lower_leg)
    print(f"Forward (x, y, z): {x2}, {y2}, {z2}")