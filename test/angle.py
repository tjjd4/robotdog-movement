import time

import sys
import os

move_logic_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(move_logic_path)
print(sys.path)

from move_logic.quadruped import Robotdog, Motor

TESTED_LEG_SHOULDER = Motor.FR_SHOULDER
TESTED_LEG_ELBOW = Motor.FR_ELBOW
TESTED_LEG_HIP = Motor.FR_HIP


if __name__ == '__main__':
    try:
        print("---開始---")
        robotdog = Robotdog()
        robotdog.calibrate()
        
        while True:
            action = input("按下 Q/W/E 控制 Shoulder，A/S/D 控制 Elbow，Z/X/C 控制 Hip，或按 Ctrl+C 終止程式：").upper()

            if action == 'Q':
                robotdog.set_angle(TESTED_LEG_SHOULDER, robotdog.get_angle(TESTED_LEG_SHOULDER) + 10)
                print(robotdog.get_angle(TESTED_LEG_SHOULDER))
            elif action == 'W':
                robotdog.set_angle(TESTED_LEG_SHOULDER, robotdog.get_angle(TESTED_LEG_SHOULDER) - 10)
                print(robotdog.get_angle(TESTED_LEG_SHOULDER))
            elif action == 'A':
                robotdog.set_angle(TESTED_LEG_ELBOW, robotdog.get_angle(TESTED_LEG_ELBOW) + 10)
                print(robotdog.get_angle(TESTED_LEG_ELBOW))
            elif action == 'S':
                robotdog.set_angle(TESTED_LEG_ELBOW, robotdog.get_angle(TESTED_LEG_ELBOW) - 10)
                print(robotdog.get_angle(TESTED_LEG_ELBOW))
            elif action == 'Z':
                robotdog.set_angle(TESTED_LEG_HIP, robotdog.get_angle(TESTED_LEG_HIP) + 10)
                print(robotdog.get_angle(TESTED_LEG_HIP))
            elif action == 'X':
                robotdog.set_angle(TESTED_LEG_HIP, robotdog.get_angle(TESTED_LEG_HIP) - 10)
                print(robotdog.get_angle(TESTED_LEG_HIP))
            elif action == 'E' or action == 'D' or action == 'C':
                break
            else:
                print("未知指令，請輸入正確的按鍵。")

    except KeyboardInterrupt:
        print("-!終止!-")
    finally:
        print("回位")
        robotdog.calibrate()
        time.sleep(1)
        print("---結束---")
