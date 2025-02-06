import time
import numpy as np

from model.quadruped import Robotdog
from model.MotionCommand import MotionCommand
from model.types.types import BehaviorState

if __name__ == '__main__':
    robotdog = Robotdog()
    
    print("Calibrating robot dog to default position...")
    robotdog.run(MotionCommand(behavior_state=BehaviorState.REST))
    time.sleep(1)
    robotdog.activate_gyroscope()
    time.sleep(1)