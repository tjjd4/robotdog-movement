from move_logic.kinematics import inverse_kinematics

if __name__ == '__main__':
    upper_leg = 10
    lower_leg = 10
    x, y, z = 0, -15, 0
    theta_shoulder, theta_elbow, theta_hip = inverse_kinematics(x, y, z, upper_leg, lower_leg)

    print(f"Position: {x}, {y}, {z}")
    print(f"Theta Shoulder: {theta_shoulder}")
    print(f"Theta Elbow: {theta_elbow}")
    print(f"Theta Hip: {theta_hip}")

    x, y, z = 0, -10, 0
    theta_shoulder, theta_elbow, theta_hip = inverse_kinematics(x, y, z, upper_leg, lower_leg)

    print(f"Position: {x}, {y}, {z}")
    print(f"Theta Shoulder: {theta_shoulder}")
    print(f"Theta Elbow: {theta_elbow}")
    print(f"Theta Hip: {theta_hip}")