import time
import numpy as np

import sys
import os

move_logic_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(move_logic_path)
print(sys.path)

from move_logic.LegController import LegController
from move_logic.hardware.Motor import Motor
from move_logic.types.leg import LegPart

TEST_SHOULDER = Motor.FL_SHOULDER
TEST_ELBOW = Motor.FL_ELBOW
TEST_HIP = Motor.FL_HIP

def controller(momentum):
    momentum[:3] = [0, 0, 1]
    return momentum

if __name__ == '__main__':
    try:
        leg = LegController(TEST_SHOULDER, TEST_ELBOW, TEST_HIP, is_opposited=False)
        is_pos1 = True
            
        while True:
            action = input("按下 Enter 鍵來切換姿勢，或按 Ctrl+C 終止程式：")
            
            if is_pos1:
                print("pos2")
                leg.set_angle_by_module(LegPart.SHOULDER, 90)
                leg.set_angle_by_module(LegPart.ELBOW, 0)
                leg.set_angle_by_module(LegPart.HIP, 90)
                
            else:
                print("pos1")
                leg.set_angle_by_module(LegPart.SHOULDER, 180)
                leg.set_angle_by_module(LegPart.ELBOW, 90)
                leg.set_angle_by_module(LegPart.HIP, 90)
                
            is_pos1 = not is_pos1
    except KeyboardInterrupt:
        print("-!終止!-")
    finally:
        print("回位")
        leg.set_angle_by_module(LegPart.SHOULDER, 90)
        leg.set_angle_by_module(LegPart.ELBOW, 0)
        leg.set_angle_by_module(LegPart.HIP, 90)
        time.sleep(1)
        print("---結束---")