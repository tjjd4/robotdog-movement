import math

def inverse_kinematics(x: float, y: float, z: float, a1: float, a2: float):
    L = 0
    y_prime = -math.sqrt((z + L)**2 + y**2)
    thetaz = math.atan2(z + L, abs(y)) - math.atan2(L, abs(y_prime))

    c2 = (x**2 + y_prime**2 - a1**2 - a2**2) / (2 * a1 * a2)
    s2 = math.sqrt(1 - c2**2)
    theta2 = math.atan2(s2, c2)

    c1 = (x * (a1 + (a2 * c2)) + y_prime * (a2 * s2)) / (x**2 + y_prime**2)
    s1 = (y_prime * (a1 + (a2 * c2)) - x * (a2 * s2)) / (x**2 + y_prime**2)
    theta1 = math.atan2(s1, c1)

    theta_shoulder = -theta1
    theta_elbow = theta_shoulder - theta2
    theta_hip: float = 90.0

    theta_shoulder = math.degrees(theta_shoulder)
    theta_elbow = math.degrees(theta_elbow)

    return theta_shoulder, theta_elbow, theta_hip