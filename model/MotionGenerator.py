import bezier
import numpy as np
from utils.ConfigHelper import ConfigHelper

class MotionGenerator:
    robotdog_config = ConfigHelper.get_section("robotdog_parameters")
    TRAJECTORY_CYCLE = robotdog_config.getint("trajectory_cycle")
    HALF_CYCLE = TRAJECTORY_CYCLE // 2
    # bezier curve reference points
    STEP_POINT_1 = [-1.0, -1.0, -15.0]
    STEP_POINT_2 = [-1.0, -1.0, -10.0]
    STEP_POINT_3 = [1.0, 1.0, -10.0]
    STEP_POINT_4 = [1.0, 1.0, -15.0]

    SLIDE_POINT_1 = [1.0, 1.0, -15.0]
    SLIDE_POINT_2 = [-1.0, -1.0, -15.0]

    @classmethod
    def generate_motion(cls):
        s_vals = np.linspace(0.0, 1.0, cls.HALF_CYCLE)

        # Combine step points into array
        step_nodes = np.asfortranarray([
            cls.STEP_POINT_1,
            cls.STEP_POINT_2,
            cls.STEP_POINT_3,
            cls.STEP_POINT_4,
        ]).T

        # Create step curve
        step_curve = bezier.Curve(step_nodes, degree=3)
        step = step_curve.evaluate_multi(s_vals)

        # Combine slide points into array
        slide_nodes = np.asfortranarray([
            cls.SLIDE_POINT_1,
            cls.SLIDE_POINT_2,
        ]).T

        # Create slide curve
        slide_curve = bezier.Curve(slide_nodes, degree=1)
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

if __name__ == "__main__":
    motion = MotionGenerator.generate_motion()
    print("Generated Motion:")
    print(motion)

    original_points = MotionGenerator.get_original_points()
    print("\nOriginal Points:")
    print(original_points)
