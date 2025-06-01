from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize(
        [
            "src/robotdog.py",
            "src/movement_executor.py",
            "src/behavior_state.py",
            "src/pose.py",
            "src/gait_generator.py",
            "src/motion_generator.py",
            "src/foot_positions.py",
            "src/position.py",
            "src/leg_position.py",
            "src/motion_command.py",
        ],
        compiler_directives={'language_level': "3"},
        build_dir="build"
    ),
)
