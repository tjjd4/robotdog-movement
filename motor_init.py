from adafruit_servokit import ServoKit
import adafruit_motor.servo
import time

def init_pose(kit):
    kit.servo[1].angle = 90
    kit.servo[2].angle = 90
    kit.servo[3].angle = 180
    kit.servo[4].angle = 90
    kit.servo[5].angle = 90
    kit.servo[6].angle = 180
    kit.servo[7].angle = 90
    kit.servo[8].angle = 90
    kit.servo[9].angle = 14
    kit.servo[10].angle = 110
    kit.servo[11].angle = 90
    kit.servo[12].angle = 14

if __name__ == '__main__':
    try:
        print("---開始---")
        kit = ServoKit(channels=16)
        init_pose(kit)
        time.sleep(1)
        print("---動作結束---")
        time.sleep(1)
    except KeyboardInterrupt:
        print("終止")
    finally:
        init_pose(kit)
        time.sleep(1)
        print("---結束---")