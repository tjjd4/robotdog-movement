import time

import sys
import os

move_logic_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(move_logic_path)
print(sys.path)

from move_logic.quadruped import Robotdog, Motor

def pos1(self):
    self.set_angle(Motor.FL_HIP, 90)
    self.set_angle(Motor.FL_SHOULDER, 90)
    self.set_angle(Motor.FL_ELBOW, 180)
    self.set_angle(Motor.FR_HIP, 90)
    self.set_angle(Motor.FR_SHOULDER, 90)
    self.set_angle(Motor.FR_ELBOW, 0)
    self.set_angle(Motor.BL_HIP, 90)
    self.set_angle(Motor.BL_SHOULDER, 90)
    self.set_angle(Motor.BL_ELBOW, 180)
    self.set_angle(Motor.BR_HIP, 90)
    self.set_angle(Motor.BR_SHOULDER, 90)
    self.set_angle(Motor.BR_ELBOW, 0)

def pos2(self):
    self.set_angle(Motor.FL_HIP, 90)
    self.set_angle(Motor.FL_SHOULDER, 0)
    self.set_angle(Motor.FL_ELBOW, 90)
    self.set_angle(Motor.FR_HIP, 90)
    self.set_angle(Motor.FR_SHOULDER, 180)
    self.set_angle(Motor.FR_ELBOW, 90)
    self.set_angle(Motor.BL_HIP, 90)
    self.set_angle(Motor.BL_SHOULDER, 0)
    self.set_angle(Motor.BL_ELBOW, 90)
    self.set_angle(Motor.BR_HIP, 90)
    self.set_angle(Motor.BR_SHOULDER, 180)
    self.set_angle(Motor.BR_ELBOW, 90)

Robotdog.pos1 = pos1
Robotdog.pos2 = pos2


if __name__ == '__main__':
    try:
        print("---開始---")
        robotdog = Robotdog()
        robotdog.pos1()
        
        # 設置初始狀態
        is_pos1 = True
        
        while True:
            action = input("按下 Enter 鍵來切換姿勢，或按 Ctrl+C 終止程式：")
            
            if is_pos1:
                print("pos2")
                robotdog.pos2()
            else:
                print("pos1")
                robotdog.pos1()
                
            is_pos1 = not is_pos1

    except KeyboardInterrupt:
        print("-!終止!-")
    finally:
        print("回位")
        robotdog.pos1()
        time.sleep(1)
        print("---結束---")