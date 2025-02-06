import curses
from curses import window
import time
import numpy as np

from model.quadruped import Robotdog
from model.MotionCommand import MotionCommand
from model.types.types import BehaviorState

def control_robot_dog(stdscr: window, robotdog: Robotdog):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0, "Control the Robot Dog:")
    stdscr.addstr(1, 0, "W: Move forward")
    stdscr.addstr(2, 0, "S: Move backward")
    stdscr.addstr(3, 0, "A: Move left")
    stdscr.addstr(4, 0, "D: Move right")
    stdscr.addstr(5, 0, "Q: Rotate counterclockwise")
    stdscr.addstr(6, 0, "E: Rotate clockwise")
    stdscr.addstr(7, 0, "R: Stand/rest")
    stdscr.addstr(8, 0, "C: Calibrate")
    stdscr.addstr(9, 0, "ESC: Exit program")

    try:
        while True:
            key = stdscr.getch()

            if key != -1:
                stdscr.clear()
                stdscr.addstr(0, 0, "Control the Robot Dog:")
                stdscr.addstr(1, 0, "W: Move forward")
                stdscr.addstr(2, 0, "S: Move backward")
                stdscr.addstr(3, 0, "A: Move left")
                stdscr.addstr(4, 0, "D: Move right")
                stdscr.addstr(5, 0, "Q: Rotate counterclockwise")
                stdscr.addstr(6, 0, "E: Rotate clockwise")
                stdscr.addstr(7, 0, "R: Stand/rest")
                stdscr.addstr(8, 0, "C: Calibrate")
                stdscr.addstr(9, 0, "ESC: Exit program")

                command = MotionCommand(horizontal_velocity=np.array([0.0, 0.0]), yaw_rate=0.0, behavior_state=BehaviorState.REST)

                if key == ord('w'):  # Forward
                    command.horizontal_velocity = np.array([1.0, 0.0])
                    command.yaw_rate = 0.0
                    command.behavior_state = BehaviorState.MOVE
                elif key == ord('s'):  # Backward
                    command.horizontal_velocity = np.array([-1.0, 0.0])
                    command.yaw_rate = 0.0
                    command.behavior_state = BehaviorState.MOVE
                elif key == ord('a'):  # Left
                    command.horizontal_velocity = np.array([0.0, 3.0])
                    command.yaw_rate = 0.0
                    command.behavior_state = BehaviorState.MOVE
                elif key == ord('d'):  # Right
                    command.horizontal_velocity = np.array([0.0, -3.0])
                    command.yaw_rate = 0.0
                    command.behavior_state = BehaviorState.MOVE
                elif key == ord('q'):  # Turn Left
                    command.horizontal_velocity = np.array([1.0, 0.0])
                    command.yaw_rate = 0.5
                    command.behavior_state = BehaviorState.MOVE
                elif key == ord('e'):  # Turn Right
                    command.horizontal_velocity = np.array([1.0, 0.0])
                    command.yaw_rate = -0.5
                    command.behavior_state = BehaviorState.MOVE
                elif key == ord('r'):  # Stand/Rest
                    command.behavior_state = BehaviorState.REST
                elif key == ord('c'):  # Calibrate
                    command.behavior_state = BehaviorState.STAND
                elif key == 27:  # ESC
                    break
                
                # 傳遞指令到機器狗
                stdscr.addstr(11, 0, f"Command: {command.behavior_state.name}, {command.horizontal_velocity}")
                stdscr.refresh()
                robotdog.run(command)
                time.sleep(0.1)

    finally:
        # 停止並校準機器狗
        robotdog.run(MotionCommand(behavior_state=BehaviorState.REST))
        time.sleep(1)
        robotdog.run(MotionCommand(behavior_state=BehaviorState.CALIBRATE))
        time.sleep(1)

if __name__ == '__main__':
    robotdog = Robotdog()
    
    print("Calibrating robot dog to default position...")
    robotdog.run(MotionCommand(behavior_state=BehaviorState.CALIBRATE))
    time.sleep(1)
    robotdog.run(MotionCommand(horizontal_velocity=np.array([0.0, 0.0]), yaw_rate=0.0, behavior_state=BehaviorState.REST))
    time.sleep(1)
    curses.wrapper(control_robot_dog, robotdog)
