from adafruit_servokit import ServoKit
import time

def init_pose(kit):
    kit.servo[10].angle = 90
    kit.servo[11].angle = 90
    kit.servo[12].angle = 90

def add_pose(kit, id):
    kit.servo[id].angle += 15

def minus_pose(kit, id):
    kit.servo[id].angle -= 15


if __name__ == '__main__':
    try:
        print("---開始---")
        kit = ServoKit(channels=16)
        init_pose(kit)
        time.sleep(1)
        while True:
            # 要求使用者輸入字母
            user_input = input("請輸入 'w' 或 's': ").strip().lower()

            # 判斷輸入的內容並執行相應操作
            if user_input == 'w':
                print("您輸入了 'w'")
                add_pose(kit, 11)
                
            elif user_input == 's':
                print("您輸入了 's'")
                minus_pose(kit, 11)
            elif user_input == 'q':
                print("您輸入了 'q'")
                add_pose(kit, 12)
                
            elif user_input == 'a':
                print("您輸入了 'a'")
                minus_pose(kit, 12)
            elif user_input == 'c':
                print("end")
                break
            else:
                print("無效輸入，請輸入 'w' 或 's' 'q' 或 'a'")
    except KeyboardInterrupt:
        print("-!終止!-")
    finally:
        print("回位")
        init_pose(kit)
        time.sleep(1)
        print("---結束---")