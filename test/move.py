import time
import numpy as np

import sys
import os

move_logic_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(move_logic_path)
print(sys.path)

from move_logic.quadruped import Robotdog

def controller(momentum):
    momentum[:3] = [0, 0, 1]
    return momentum


if __name__ == '__main__':
    robotdog = Robotdog()
    
    print("Calibrating robot dog to default position...")
    robotdog.calibrate()
    action = input("press 'Enter' to -> Standup")
    robotdog.calibrate_by_inverse_positioning()
    action = input("press 'Enter' to start ->")

    momentum = np.asarray([0, 0, 1, 0], dtype=np.float32)  # 最後一個值為 0 表示不關閉

    print("Starting robot dog movement. Press Ctrl+C to stop.")
    try:
        robotdog.move(controller)  # 使用 lambda 傳遞 momentum 給 controller
    except KeyboardInterrupt:
        print("\nCtrl+C detected! Sending stop signal to robot dog...")

        momentum[3] = 1
        time.sleep(1)
        robotdog.calibrate()
        print("Movement stopped by user. Exiting program.")