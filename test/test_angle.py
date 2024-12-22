import time

import sys
import os

move_logic_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(move_logic_path)
print(sys.path)

from move_logic.LegController import LegController
from move_logic.hardware.Motor import Motor
from move_logic.types.leg import LegPart

TESTED_LEG_SHOULDER = Motor.FL_SHOULDER
TESTED_LEG_ELBOW = Motor.FL_ELBOW
TESTED_LEG_HIP = Motor.FL_HIP
TEST_IS_OPPOSITED = False


if __name__ == '__main__':
    try:
        print("---開始---")

        test_leg = LegController(TESTED_LEG_SHOULDER, TESTED_LEG_ELBOW, TESTED_LEG_HIP, is_opposited=TEST_IS_OPPOSITED)

        test_leg.pose1()
        
        while True:
            action = input("按下 Q/W/E 控制 Shoulder，A/S/D 控制 Elbow，Z/X/C 控制 Hip，或按 Ctrl+C 終止程式：").upper()

            if action == 'Q':
                test_leg.set_shoulder_angle(test_leg.get_angle_by_module(LegPart.SHOULDER) + 10)
                print(test_leg.get_angle_by_module(LegPart.SHOULDER))
            elif action == 'W':
                test_leg.set_shoulder_angle(test_leg.get_angle_by_module(LegPart.SHOULDER) - 10)
                print(test_leg.get_angle_by_module(LegPart.SHOULDER))
            elif action == 'A':
                test_leg.set_elbow_angle(test_leg.get_angle_by_module(LegPart.ELBOW) + 10)
                print(test_leg.get_angle_by_module(LegPart.ELBOW))
            elif action == 'S':
                test_leg.set_elbow_angle(test_leg.get_angle_by_module(LegPart.ELBOW) - 10)
                print(test_leg.get_angle_by_module(LegPart.ELBOW))
            elif action == 'Z':
                test_leg.set_hip_angle(test_leg.get_angle_by_module(LegPart.HIP) + 10)
                print(test_leg.get_angle_by_module(LegPart.HIP))
            elif action == 'X':
                test_leg.set_hip_angle(test_leg.get_angle_by_module(LegPart.HIP) - 10)
                print(test_leg.get_angle_by_module(LegPart.HIP))
            elif action == 'E' or action == 'D' or action == 'C':
                break
            else:
                print("未知指令，請輸入正確的按鍵。")

    except KeyboardInterrupt:
        print("-!終止!-")
    finally:
        print("回位")
        test_leg.pose1()
        time.sleep(1)
        print("---結束---")