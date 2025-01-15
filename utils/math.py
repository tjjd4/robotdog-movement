from typing import Literal
import numpy as np
from transforms3d.euler import euler2mat

def get_plane_from_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray):
    v1 = np.array(p2) - np.array(p1)
    v2 = np.array(p3) - np.array(p1)

    # 計算法向量（叉積）
    normal = np.cross(v1, v2)
    A, B, C = normal

    # 計算 D
    D = -(A * p1[0] + B * p1[1] + C * p1[2])

    return A,B,C,D

def turn_points_with_euler_radians(points: np.ndarray, radians: float, axis: Literal["x","y","z"]="x"):
    if points.shape[0] != 3:
        raise ValueError("input points must have 3 dimension (x, z, y)")

    if axis == "x":
        rotation_matrix = euler2mat(radians, 0, 0)
    elif axis == "y":
        rotation_matrix = euler2mat(0, radians, 0)
    elif axis == "z":
        rotation_matrix = euler2mat(0, 0, radians)
    
    # 應用旋轉
    rotated_points = rotation_matrix @ points
    return rotated_points