from threading import Lock

from src.model.custom_types.index import MotionCommand, FootPositions, RobotDogState


class StateManager:
    # should be attribute name of RobotDogState and MotionCommand
    UPDATE_ALLOWED_FIELDS = {
        'horizontal_velocity', 'yaw_rate', 'roll', 'pitch',
        'yaw', 'height', 'behavior_state', 'is_gyro_activated'
    }

    def __init__(self, state: RobotDogState):
        self._state: RobotDogState = state
        self._lock = Lock()

    def update_state_by_motion_command(self, command: MotionCommand):
        if command is not None:
            with self._lock:
                for field_name in self.UPDATE_ALLOWED_FIELDS:
                    value = getattr(command, field_name)
                    if value is not None:
                        setattr(self._state, field_name, value)

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
        
    def get_delay_time(self):
        with self._lock:
            return self._state.delay_time

    def get_yaw_rate(self):
        with self._lock:
            return self._state.yaw_rate
        
