import numpy as np
import math
import matplotlib.pyplot as plt
from transforms3d.euler import euler2mat

import sys
import os

move_logic_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(move_logic_path)

from move_logic.motion_generator import generate_motion

# 生成原始軌跡
def generate_original_trajectory():
    motion = generate_motion()
    trajectory = motion * np.array([1, 0, 1])[:, None]
    return trajectory

# 旋轉軌跡
def rotate_trajectory(trajectory, yaw_rate):
    yaw_angle = math.radians(yaw_rate)
    rotation_matrix = euler2mat(yaw_angle, 0, 0)
    return rotation_matrix @ trajectory

yaw_rate = 10

# 生成軌跡
original_trajectory = generate_original_trajectory()
rotated_trajectory = rotate_trajectory(original_trajectory, yaw_rate)

# 視覺化
fig, axes = plt.subplots(1, 2, figsize=(12, 6))

# 左圖：z = 0 的情況
axes[0].plot(original_trajectory[0], original_trajectory[1], 'r--', label='Original')
axes[0].plot(rotated_trajectory[0], rotated_trajectory[1], 'b-', label='Rotated')
axes[0].set_title('Yaw rotation with z = 0')
axes[0].set_xlabel('X (forward)')
axes[0].set_ylabel('Z (left/right)')
axes[0].legend()
axes[0].grid()
axes[0].set_aspect('equal', adjustable='box')

# 右圖：z ≠ 0 的情況
axes[1].plot(original_trajectory[0], original_trajectory[2], 'r--', label='Original')
axes[1].plot(rotated_trajectory[0], rotated_trajectory[2], 'b-', label='Rotated')
axes[1].set_title('Yaw rotation with z ≠ 0')
axes[1].set_xlabel('X (forward)')
axes[1].set_ylabel('Y (up/down)')
axes[1].legend()
axes[1].grid()
axes[1].set_aspect('equal', adjustable='box')

# 調整圖形佈局
plt.tight_layout()
plt.show()
