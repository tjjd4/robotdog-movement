import time

from model.quadruped import Robotdog
from model.types.leg import LegPosition, LegPart

def pos1(self):
    self.set_angle(LegPosition.FL, LegPart.SHOULDER, 180)
    self.set_angle(LegPosition.FL, LegPart.ELBOW, 90)
    self.set_angle(LegPosition.FL, LegPart.HIP, 90)

    self.set_angle(LegPosition.FR, LegPart.SHOULDER, 180)
    self.set_angle(LegPosition.FR, LegPart.ELBOW, 90)
    self.set_angle(LegPosition.FR, LegPart.HIP, 90)

    self.set_angle(LegPosition.BL, LegPart.SHOULDER, 180)
    self.set_angle(LegPosition.BL, LegPart.ELBOW, 90)
    self.set_angle(LegPosition.BL, LegPart.HIP, 90)

    self.set_angle(LegPosition.BR, LegPart.SHOULDER, 180)
    self.set_angle(LegPosition.BR, LegPart.ELBOW, 90)
    self.set_angle(LegPosition.BR, LegPart.HIP, 90)

def pos2(self):
    self.set_angle(LegPosition.FL, LegPart.SHOULDER, 90)
    self.set_angle(LegPosition.FL, LegPart.ELBOW, 0)
    self.set_angle(LegPosition.FL, LegPart.HIP, 90)

    self.set_angle(LegPosition.FR, LegPart.SHOULDER, 90)
    self.set_angle(LegPosition.FR, LegPart.ELBOW, 0)
    self.set_angle(LegPosition.FR, LegPart.HIP, 90)

    self.set_angle(LegPosition.BL, LegPart.SHOULDER, 90)
    self.set_angle(LegPosition.BL, LegPart.ELBOW, 0)
    self.set_angle(LegPosition.BL, LegPart.HIP, 90)

    self.set_angle(LegPosition.BR, LegPart.SHOULDER, 90)
    self.set_angle(LegPosition.BR, LegPart.ELBOW, 0)
    self.set_angle(LegPosition.BR, LegPart.HIP, 90)

Robotdog.pos1 = pos1
Robotdog.pos2 = pos2


if __name__ == '__main__':
    try:
        print("---開始---")
        robotdog = Robotdog()
        robotdog.calibrate()
        
        # 設置初始狀態
        is_pos1 = False
        
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
        robotdog.calibrate()
        time.sleep(1)
        print("---結束---")