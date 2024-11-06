import importlib
import argparse

# from controllers.local_keyboard_controller import controller
from move_logic.quadruped import Robotdog

def get_controller(controller_file):
    controller_lib = importlib.import_module(controller_file)
    return controller_lib.controller

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--controller', default='controllers.network_receiver')
    args = parser.parse_args()

    controller = get_controller(args.controller)

    r = Robotdog()
    r.calibrate()
    r.move(controller)