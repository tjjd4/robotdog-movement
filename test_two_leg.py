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
    kit.servo[8].angle = 110
    kit.servo[9].angle = 5
    kit.servo[10].angle = 110
    kit.servo[11].angle = 110
    kit.servo[12].angle = 0

def rear_leg_up(kit):
    kit.servo[5].angle += 30
    kit.servo[11].angle -= 30

def rear_leg_down(kit):
    kit.servo[5].angle -= 30
    kit.servo[11].angle += 30

def front_leg_up(kit):
    kit.servo[2].angle += 30
    kit.servo[8].angle -= 30

def front_leg_down(kit):
    kit.servo[2].angle -= 30
    kit.servo[8].angle += 30


if __name__ == '__main__':
    try:
        print("---開始---")
        kit = ServoKit(channels=16)
        init_pose(kit)
        time.sleep(1)
        for channel in range(2):
            kit.servo[channel].angle
            print(f"測試 {channel}號 servo，目前{kit.servo[channel].angle} 度")
            front_leg_up(kit)
            time.sleep(1)
            rear_leg_up(kit)
            time.sleep(5)
            rear_leg_down(kit)
            time.sleep(1)
            front_leg_down(kit)
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