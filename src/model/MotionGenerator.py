import bezier
import numpy as np
from src.model.custom_types.index import Position
from src.utils.ConfigHelper import ConfigHelper

class MotionGenerator:
    movement_config = ConfigHelper.get_section("movement_parameters")
    TRAJECTORY_CYCLE = movement_config.getint("trajectory_cycle", fallback=40)
    MAX_HEIGHT = movement_config.getfloat("max_height", fallback=15.0)
    MIN_HEIGHT = movement_config.getfloat("min_height", fallback=10.0)
    HALF_CYCLE = TRAJECTORY_CYCLE // 2
    # bezier curve reference points
    STEP_POINT_1 = [-1.0, -1.0, -MAX_HEIGHT]
    STEP_POINT_2 = [-1.0, -1.0, -MIN_HEIGHT]
    STEP_POINT_3 = [1.0, 1.0, -MAX_HEIGHT]
    STEP_POINT_4 = [1.0, 1.0, -MAX_HEIGHT]

    SLIDE_POINT_1 = [1.0, 1.0, -MAX_HEIGHT]
    SLIDE_POINT_2 = [-1.0, -1.0, -MAX_HEIGHT]

    STEP_NODES = np.asfortranarray([
        STEP_POINT_1,
        STEP_POINT_2,
        STEP_POINT_3,
        STEP_POINT_4,
    ]).T

    SLIDE_NODES = np.asfortranarray([
        SLIDE_POINT_1,
        SLIDE_POINT_2,
    ]).T

    @classmethod
    def generate_motion(cls):
        s_vals = np.linspace(0.0, 1.0, cls.HALF_CYCLE)

        # Create step curve
        step_curve = bezier.Curve(cls.STEP_NODES, degree=3)
        step = step_curve.evaluate_multi(s_vals)

        # Create slide curve
        slide_curve = bezier.Curve(cls.SLIDE_NODES, degree=1)
        slide = slide_curve.evaluate_multi(s_vals)

        # Combine step and slide motions
        return np.concatenate((step, slide), axis=1)

    @classmethod
    def get_original_points(cls):
        step_nodes = np.asfortranarray([
            cls.STEP_POINT_1,
            cls.STEP_POINT_2,
            cls.STEP_POINT_3,
            cls.STEP_POINT_4,
        ]).T

        return step_nodes

    @classmethod
    def generate_motion_from_position(cls, position: Position):
        motion = cls.generate_motion()
        np_position_offset = np.asfortranarray([[position.x], [position.z], [position.y + cls.MAX_HEIGHT]])
        adjust_motion = motion + np_position_offset
        return adjust_motion


if __name__ == "__main__":
    print("STEP NODES:\n")
    print("Shape:\n", MotionGenerator.STEP_NODES.shape)
    motion = MotionGenerator.generate_motion()
    print("Generated Motion:\n")
    print("Shape:\n", motion.shape)
    print(motion)

    original_points = MotionGenerator.get_original_points()
    print("\nOriginal Points:\n")
    print(original_points)

    x, y, z = (1, -15, 0)
    position = Position(x=x, y=y, z=z)

    motion_from_position = MotionGenerator.generate_motion_from_position(position)
    print(f"\nMotion from Positions: {position}\n")
    print("Shape:\n", motion_from_position.shape)
    print(motion_from_position)
