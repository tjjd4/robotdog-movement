import math

def inverse_kinematics(x: float, y: float, z: float, a1: float, a2: float):
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