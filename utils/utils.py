import numpy as np
from typing import Literal

from model.types.types import LegsPositions, Position


def get_np_array_from_legs_positions(legs_positions: LegsPositions, order: Literal['xyz', 'xzy']='xzy') -> np.ndarray:
    """
    Convert LegsPositions into a NumPy array with specified axis order.
    
    Args:
        legs_positions (LegsPositions): The structure containing leg positions.
        order (Literal['xyz', 'xzy']): Determines the order of the axes.
    
    Returns:
        np.ndarray: A (3, 4) array where rows represent x, y, z coordinates and columns represent FL, FR, BL, BR.
    """
    # Extract the x, y, z coordinates into separate lists
    x = np.array([legs_positions.FL.x, legs_positions.FR.x, legs_positions.BL.x, legs_positions.BR.x])
    y = np.array([legs_positions.FL.y, legs_positions.FR.y, legs_positions.BL.y, legs_positions.BR.y])
    z = np.array([legs_positions.FL.z, legs_positions.FR.z, legs_positions.BL.z, legs_positions.BR.z])

    if order == 'xyz':
        return np.vstack([x, y, z])
    elif order == 'xzy':
        return np.vstack([x, z, y])
    else:
        raise ValueError("Invalid order. Use 'xyz' or 'xzy'.")

if __name__ == '__main__':

    legs_positions = LegsPositions(
        FL=Position(-10, -14.54609376, -7.5),
        FR=Position(-10, -11.98094269, 7.5),
        BL=Position(10, -18.01905731, -7.5),
        BR=Position(10, -15.45390624, 7.5)
    )

    result = get_np_array_from_legs_positions(legs_positions)
    print(result)