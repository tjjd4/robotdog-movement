import time
import numpy as np

from model.quadruped import Robotdog
from model.types.types import MotionCommand
from model.types.types import BehaviorState



if __name__ == '__main__':
    try:
        print("---開始---")
        robotdog = Robotdog()
        robotdog.calibrate()
        
        time.sleep(1)
        # 設置初始狀態
        is_gyro_activated = False

        command = MotionCommand(horizontal_velocity=np.array([0.0, 0.0]), yaw_rate=0.0, behavior_state=BehaviorState.STAND)

        robotdog.run(command)
        
        while True:
            action = input(f"按下 Enter 鍵來啟用和停止陀螺儀，或按 Ctrl+C 終止程式： 目前狀態為 {'開啟(ON)' if is_gyro_activated else '關閉(OFF)'}")
            
            if is_gyro_activated:
                print("關閉")
                robotdog.deactivate_gyroscope()
            else:
                print("開啟")
                robotdog.activate_gyroscope()
                
            is_gyro_activated = not is_gyro_activated

    except KeyboardInterrupt:
        print("-!終止!-")
    finally:
        print("回位")
        if is_gyro_activated:
            robotdog.deactivate_gyroscope()
        robotdog.calibrate()
        time.sleep(1)
        print("---結束---")