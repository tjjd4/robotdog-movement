import time
import numpy as np
import logging

from src.model.Robotdog import Robotdog
from src.model.custom_types.index import MotionCommand, BehaviorState

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def test_fixed_movements(robotdog: Robotdog):
    actions = [
        ("前進", MotionCommand(horizontal_velocity=np.array([3.0, 0.0]), yaw_rate=0.0, behavior_state=BehaviorState.MOVE)),
        ("後退", MotionCommand(horizontal_velocity=np.array([-3.0, 0.0]), yaw_rate=0.0, behavior_state=BehaviorState.MOVE)),
        ("左轉", MotionCommand(horizontal_velocity=np.array([2.0, 0.0]), yaw_rate=0.5, behavior_state=BehaviorState.MOVE)),
        ("右轉", MotionCommand(horizontal_velocity=np.array([2.0, 0.0]), yaw_rate=-0.5, behavior_state=BehaviorState.MOVE)),
        ("靜止", MotionCommand(horizontal_velocity=np.array([0.0, 0.0]), yaw_rate=0.0, behavior_state=BehaviorState.REST)),
    ]

    for label, cmd in actions:
        logger.info(f"\n▶ 執行動作：{label}")
        robotdog.run(cmd)
        time.sleep(2)

if __name__ == '__main__':
    robotdog = Robotdog()
    test_fixed_movements(robotdog)