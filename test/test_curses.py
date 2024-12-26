import curses
import time
import numpy as np

import sys
import os

move_logic_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(move_logic_path)

from move_logic.MotionCommand import MotionCommand

def control_test(stdscr):
    """測試 curses 交互功能"""
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0, "Test Curses Interface:")
    stdscr.addstr(1, 0, "W: Move forward")
    stdscr.addstr(2, 0, "S: Move backward")
    stdscr.addstr(3, 0, "A: Move left")
    stdscr.addstr(4, 0, "D: Move right")
    stdscr.addstr(5, 0, "Q: Rotate counterclockwise")
    stdscr.addstr(6, 0, "E: Rotate clockwise")
    stdscr.addstr(7, 0, "R: Rest")
    stdscr.addstr(8, 0, "ESC: Exit program")

    command = MotionCommand()

    try:
        while True:
            key = stdscr.getch()
            stdscr.clear()
            stdscr.addstr(0, 0, "Test Curses Interface:")
            stdscr.addstr(1, 0, "W: Move forward")
            stdscr.addstr(2, 0, "S: Move backward")
            stdscr.addstr(3, 0, "A: Move left")
            stdscr.addstr(4, 0, "D: Move right")
            stdscr.addstr(5, 0, "Q: Rotate counterclockwise")
            stdscr.addstr(6, 0, "E: Rotate clockwise")
            stdscr.addstr(7, 0, "R: Rest")
            stdscr.addstr(8, 0, "ESC: Exit program")

            if key == ord('w'):  # Forward
                command.horizontal_velocity = np.array([1.0, 0.0])
                command.behavior_state = "MOVE"
            elif key == ord('s'):  # Backward
                command.horizontal_velocity = np.array([-1.0, 0.0])
                command.behavior_state = "MOVE"
            elif key == ord('a'):  # Left
                command.horizontal_velocity = np.array([0.0, 1.0])
                command.behavior_state = "MOVE"
            elif key == ord('d'):  # Right
                command.horizontal_velocity = np.array([0.0, -1.0])
                command.behavior_state = "MOVE"
            elif key == ord('q'):  # Rotate counterclockwise
                command.yaw_rate = 0.5
                command.behavior_state = "MOVE"
            elif key == ord('e'):  # Rotate clockwise
                command.yaw_rate = -0.5
                command.behavior_state = "MOVE"
            elif key == ord('r'):  # Rest
                command.behavior_state = "REST"
            elif key == 27:  # ESC
                break


            # 模擬輸出當前命令狀態
            stdscr.addstr(10, 0, f"Motion Command Output:")
            stdscr.addstr(11, 0, f"Horizontal Velocity: {command.horizontal_velocity}")
            stdscr.addstr(12, 0, f"Yaw Rate: {command.yaw_rate}")
            stdscr.addstr(13, 0, f"Behavior State: {command.behavior_state}")

            time.sleep(0.1)

    finally:
        stdscr.addstr(14, 0, "Exiting...")

if __name__ == '__main__':
    curses.wrapper(control_test)
