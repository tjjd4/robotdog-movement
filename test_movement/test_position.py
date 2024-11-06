from adafruit_servokit import ServoKit
import adafruit_motor.servo
import time
from enum import IntEnum

class Motor(IntEnum):
    # identifies the corresponding pin location with the motor location
    FR_HIP = 1
    FR_SHOULDER = 2
    FR_ELBOW = 3
    FL_HIP = 4
    FL_SHOULDER = 5
    FL_ELBOW = 6
    BR_HIP = 7
    BR_SHOULDER = 8
    BR_ELBOW = 9
    BL_HIP = 10
    BL_SHOULDER = 11
    BL_ELBOW = 12

def init_pose(kit):
    kit.servo[1].angle = 90
    kit.servo[2].angle = 90
    kit.servo[3].angle = 60
    kit.servo[4].angle = 90
    kit.servo[5].angle = 90
    kit.servo[6].angle = 120
    kit.servo[7].angle = 90
    kit.servo[8].angle = 90
    kit.servo[9].angle = 60
    kit.servo[10].angle = 90
    kit.servo[11].angle = 90
    kit.servo[12].angle = 120

def standup(kit):
    kit.servo[2].angle += 60
    kit.servo[3].angle -= 20
    kit.servo[5].angle += 60
    kit.servo[6].angle -= 20
    kit.servo[8].angle -= 60
    kit.servo[9].angle += 20
    kit.servo[11].angle -= 60
    kit.servo[12].angle += 20

def down(kit):
    kit.servo[2].angle -= 60
    kit.servo[3].angle += 20
    kit.servo[5].angle -= 60
    kit.servo[6].angle += 20
    kit.servo[8].angle += 60
    kit.servo[9].angle -= 20
    kit.servo[11].angle += 60
    kit.servo[12].angle -= 20

if __name__ == '__main__':
    try:
        print("---開始---")
        kit = ServoKit(channels=16)
        init_pose(kit)
        time.sleep(1)
        for channel in range(1, 13):
            print(f"站立")
            # standup(kit)
            time.sleep(5)
            print(f"蹲")
            # down(kit)
            time.sleep(1)
        print("---動作結束---")
        time.sleep(1)
    except KeyboardInterrupt:
        print("-!終止!-")
    finally:
        print("回位")
        init_pose(kit)
        time.sleep(1)
        print("---結束---")