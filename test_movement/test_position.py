from adafruit_servokit import ServoKit
import adafruit_motor.servo
import time
from enum import IntEnum

class Motor(IntEnum):
    # identifies the corresponding pin location with the motor location
    FL_HIP = 1
    FL_SHOULDER = 2
    FL_ELBOW = 3
    BL_HIP = 4
    BL_SHOULDER = 5
    BL_ELBOW = 6
    FR_HIP = 7
    FR_SHOULDER = 8
    FR_ELBOW = 9
    BR_HIP = 10
    BR_SHOULDER = 11
    BR_ELBOW = 12



def init_pose(kit):
    kit.servo[Motor.FL_HIP].angle = 90
    kit.servo[Motor.FL_SHOULDER].angle = 0
    kit.servo[Motor.FL_ELBOW].angle = 160
    kit.servo[Motor.FR_HIP].angle = 90
    kit.servo[Motor.FR_SHOULDER].angle = 180
    kit.servo[Motor.FR_ELBOW].angle = 20

    kit.servo[Motor.BL_HIP].angle = 90
    kit.servo[Motor.BL_SHOULDER].angle = 0
    kit.servo[Motor.BL_ELBOW].angle = 160
    kit.servo[Motor.BR_HIP].angle = 90
    kit.servo[Motor.BR_SHOULDER].angle = 180
    kit.servo[Motor.BR_ELBOW].angle = 20
    

def standup(kit):
    kit.servo[Motor.FL_HIP].angle = 90
    kit.servo[Motor.FL_SHOULDER].angle = 70
    kit.servo[Motor.FL_ELBOW].angle = 110
    kit.servo[Motor.FR_HIP].angle = 90
    kit.servo[Motor.FR_SHOULDER].angle = 110
    kit.servo[Motor.FR_ELBOW].angle = 70

    kit.servo[Motor.BL_HIP].angle = 90
    kit.servo[Motor.BL_SHOULDER].angle = 70
    kit.servo[Motor.BL_ELBOW].angle = 130
    kit.servo[Motor.BR_HIP].angle = 90
    kit.servo[Motor.BR_SHOULDER].angle = 110
    kit.servo[Motor.BR_ELBOW].angle = 50

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
        for _ in range(1, 13):
            print(f"站立")
            standup(kit)
            time.sleep(3)
            print(f"回位")
            init_pose(kit)
            time.sleep(1)
        time.sleep(1)
    except KeyboardInterrupt:
        print("-!終止!-")
    finally:
        print("回位")
        init_pose(kit)
        time.sleep(1)
        print("---結束---")