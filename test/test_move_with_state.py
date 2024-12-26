import time
import numpy as np

import sys
import os

move_logic_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(move_logic_path)
print(sys.path)

from move_logic.quadruped import Robotdog
from move_logic.MotionCommand import MotionCommand
from move_logic.types.BehaviorState import BehaviorState

if __name__ == '__main__':
    robotdog = Robotdog()
    
    print("Calibrating robot dog to default position...")
    robotdog.run()
    action = input("press 'Enter' to start ->")

    print("Starting robot dog movement. Press Ctrl+C to stop.")
    try:
        command = MotionCommand(horizontal_velocity=np.array([1.0,0.0]), behavior_state=BehaviorState.MOVE)
        robotdog.run(command)

    except KeyboardInterrupt:
        print("\nCtrl+C detected! Sending stop signal to robot dog...")
        command = MotionCommand(behavior_state=BehaviorState.REST)
        robotdog.run(command)
        time.sleep(1)
        robotdog.calibrate()
        print("Movement stopped by user. Exiting program.")