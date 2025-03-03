import numpy as np
from typing import Literal

from model.types.types import FootPositions, Position


def get_np_array_from_foot_positions(foot_positions: FootPositions, order: Literal['xyz', 'xzy']='xzy') -> np.ndarray:
    """
    Convert FootPositions into a NumPy array with specified axis order.
    
    Args:
        foot_positions (FootPositions): The structure containing leg positions.
        order (Literal['xyz', 'xzy']): Determines the order of the axes.
    
    Returns:
        np.ndarray: A (3, 4) array where rows represent x, y, z coordinates and columns represent FL, FR, BL, BR.
    """
    # Extract the x, y, z coordinates into separate lists
    x = np.array([foot_positions.FL.x, foot_positions.FR.x, foot_positions.BL.x, foot_positions.BR.x], dtype=np.float64)
    y = np.array([foot_positions.FL.y, foot_positions.FR.y, foot_positions.BL.y, foot_positions.BR.y], dtype=np.float64)
    z = np.array([foot_positions.FL.z, foot_positions.FR.z, foot_positions.BL.z, foot_positions.BR.z], dtype=np.float64)

    if order == 'xyz':
        return np.vstack([x, y, z])
    elif order == 'xzy':
        return np.vstack([x, z, y])
    else:
        raise ValueError("Invalid order. Use 'xyz' or 'xzy'.")

def get_foot_positions_from_np_array(positions: np.ndarray, order: Literal['xyz', 'xzy']='xzy') -> FootPositions:
    if positions.shape != (3, 4):
        raise ValueError("Invalid shape for positions array. Expected (3, 4).")
    
    if order == 'xyz':
        x, y, z = positions
    elif order == 'xzy':
        x, z, y = positions
    else:
        raise ValueError("Invalid order. Use 'xyz' or 'xzy'.")
    
    return FootPositions(
        FL=Position(x[0], y[0], z[0]),
        FR=Position(x[1], y[1], z[1]),
        BL=Position(x[2], y[2], z[2]),
        BR=Position(x[3], y[3], z[3])
    )

if __name__ == '__main__':

    foot_positions = FootPositions(
        FL=Position(-10, -14.54609376, -7.5),
        FR=Position(-10, -11.98094269, 7.5),
        BL=Position(10, -18.01905731, -7.5),
        BR=Position(10, -15.45390624, 7.5)
    )

    ans1 = get_np_array_from_foot_positions(foot_positions)
    print(ans1)
    temp = get_foot_positions_from_np_array(ans1)

    ans2 = get_np_array_from_foot_positions(temp)
    print(ans2)