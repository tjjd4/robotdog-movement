import time

from model.Robotdog import Robotdog

if __name__ == '__main__':
    robotdog = Robotdog()
    try:
        print("---開始---")
        robotdog.calibrate()

        # 設置初始狀態
        is_pos1 = False

        while True:
            action = input("按下 Enter 鍵來切換姿勢，或按 Ctrl+C 終止程式：")

            if is_pos1:
                print("pos2")
                robotdog.set_four_legs_angle(shoulder_angle=180, elbow_angle=90, hip_angle=90)
            else:
                print("pos1")
                robotdog.set_four_legs_angle(shoulder_angle=90, elbow_angle=0, hip_angle=90)

            is_pos1 = not is_pos1

    except KeyboardInterrupt:
        print("-!終止!-")
    finally:
        print("回位")
        robotdog.calibrate()
        time.sleep(1)
        print("---結束---")
