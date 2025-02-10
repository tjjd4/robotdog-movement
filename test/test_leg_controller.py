import time

from model.LegController import LegController
from model.hardware.Motor import Motor
from model.types.types import LegPart

TEST_SHOULDER = Motor.FL_SHOULDER
TEST_ELBOW = Motor.FL_ELBOW
TEST_HIP = Motor.FL_HIP

def controller(momentum):
    momentum[:3] = [0, 0, 1]
    return momentum

if __name__ == '__main__':
    try:
        leg = LegController(TEST_SHOULDER, TEST_ELBOW, TEST_HIP, FB_is_opposited=False, LR_is_opposited=False)
        is_pos1 = True
            
        while True:
            action = input("按下 Enter 鍵來切換姿勢，或按 Ctrl+C 終止程式：")
            
            if is_pos1:
                print("pos2")
                leg.set_angle(LegPart.SHOULDER, 90)
                leg.set_angle(LegPart.ELBOW, 0)
                leg.set_angle(LegPart.HIP, 90)
                
            else:
                print("pos1")
                leg.set_angle(LegPart.SHOULDER, 180)
                leg.set_angle(LegPart.ELBOW, 90)
                leg.set_angle(LegPart.HIP, 90)
                
            is_pos1 = not is_pos1
    except KeyboardInterrupt:
        print("-!終止!-")
    finally:
        print("回位")
        leg.set_angle(LegPart.SHOULDER, 90)
        leg.set_angle(LegPart.ELBOW, 0)
        leg.set_angle(LegPart.HIP, 90)
        time.sleep(1)
        print("---結束---")