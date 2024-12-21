import bezier
import numpy as np

def generate_motion():
    s_vals = np.linspace(0.0, 1.0, 20)
    step_nodes = np.asfortranarray([
        [-1.0, -1.0, 1.0, 1.0],
        [-1.0, -1.0, 1.0, 1.0],
        [-15.0, -10, -10, -15.0],
    ])
    curve = bezier.Curve(step_nodes, degree=3)
    step = curve.evaluate_multi(s_vals)

    slide_nodes = np.asfortranarray([
        [1.0, -1.0],
        [1.0, -1.0],
        [-15.0, -15],
    ])
    curve = bezier.Curve(slide_nodes, degree=1)
    slide = curve.evaluate_multi(s_vals)

    return np.concatenate((step, slide), axis=1)