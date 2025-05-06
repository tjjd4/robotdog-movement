import time
import numpy as np

from src.model.Robotdog import Robotdog
from src.model.custom_types.index import MotionCommand, BehaviorState

def test_fixed_movements(robotdog: Robotdog):
    actions = [
        ("前進", MotionCommand(horizontal_velocity=np.array([3.0, 0.0]), yaw_rate=0.0, behavior_state=BehaviorState.MOVE)),
        ("後退", MotionCommand(horizontal_velocity=np.array([-3.0, 0.0]), yaw_rate=0.0, behavior_state=BehaviorState.MOVE)),
        ("左移", MotionCommand(horizontal_velocity=np.array([0.0, 3.0]), yaw_rate=0.0, behavior_state=BehaviorState.MOVE)),
        ("右移", MotionCommand(horizontal_velocity=np.array([0.0, -3.0]), yaw_rate=0.0, behavior_state=BehaviorState.MOVE)),
        ("左轉", MotionCommand(horizontal_velocity=np.array([1.0, 0.0]), yaw_rate=0.5, behavior_state=BehaviorState.MOVE)),
        ("右轉", MotionCommand(horizontal_velocity=np.array([1.0, 0.0]), yaw_rate=-0.5, behavior_state=BehaviorState.MOVE)),
        ("靜止", MotionCommand(horizontal_velocity=np.array([0.0, 0.0]), yaw_rate=0.0, behavior_state=BehaviorState.REST)),
    ]

    for label, cmd in actions:
        print(f"\n▶ 執行動作：{label}")
        robotdog.run(cmd)
        time.sleep(2)

if __name__ == '__main__':
    robotdog = Robotdog()
    test_fixed_movements(robotdog)