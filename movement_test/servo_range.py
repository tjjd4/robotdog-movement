from adafruit_servokit import ServoKit
import time

def init_pose(kit):
    kit.servo[1].angle = 90
    kit.servo[2].angle = 90
    kit.servo[3].angle = 90
    kit.servo[4].angle = 90
    kit.servo[5].angle = 90
    kit.servo[6].angle = 90
    kit.servo[7].angle = 90
    kit.servo[8].angle = 90
    kit.servo[9].angle = 90
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

            # 判斷輸入的內容並執行相應操作 ws qa ed rf, ik, ol, uj, yh
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
            elif user_input == 'e':
                print("您輸入了 'e'")
                add_pose(kit, 8)
                
            elif user_input == 'd':
                print("您輸入了 'd'")
                minus_pose(kit, 8)
            elif user_input == 'r':
                print("您輸入了 'r'")
                add_pose(kit, 9)
                
            elif user_input == 'f':
                print("您輸入了 'f'")
                minus_pose(kit, 9)

            elif user_input == 'i':
                print("您輸入了 'i'")
                add_pose(kit, 2)
            elif user_input == 'k':
                print("您輸入了 'k'")
                minus_pose(kit, 2)

            elif user_input == 'o':
                print("您輸入了 'o'")
                add_pose(kit, 3)
            elif user_input == 'l':
                print("您輸入了 'l'")
                minus_pose(kit, 3)

            elif user_input == 'u':
                print("您輸入了 'u'")
                add_pose(kit, 5)
            elif user_input == 'j':
                print("您輸入了 'j'")
                minus_pose(kit, 5)

            elif user_input == 'y':
                print("您輸入了 'y'")
                add_pose(kit, 6)
            elif user_input == 'h':
                print("您輸入了 'h'")
                minus_pose(kit, 6)

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