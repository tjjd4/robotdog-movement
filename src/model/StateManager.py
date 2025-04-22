from threading import Lock

from src.model.custom_types.index import MotionCommand, FootPositions, RobotDogState


class StateManager:
    def __init__(self, state: RobotDogState):
        self._state: RobotDogState = state
        self._lock = Lock()

    def update_state_by_motion_command(self, command: MotionCommand):
        with self._lock:
            self._state.horizontal_velocity = command.horizontal_velocity
            self._state.yaw_rate = command.yaw_rate
            self._state.roll = command.roll
            self._state.pitch = command.pitch
            self._state.yaw = command.yaw
            self._state.height = command.height
            self._state.behavior_state = command.behavior_state

    def get_state(self):
        with self._lock:
            return self._state

    def set_foot_positions(self, foot_positions: FootPositions):
        with self._lock:
            self._state.foot_current_positions = foot_positions

    def get_foot_positions(self) -> FootPositions:
        with self._lock:
            return self._state.foot_current_positions

    def get_behavior_state(self):
        with self._lock:
            return self._state.behavior_state

    def get_horizontal_velocity(self):
        with self._lock:
            return self._state.horizontal_velocity

    def get_height(self):
        with self._lock:
            return self._state.height
