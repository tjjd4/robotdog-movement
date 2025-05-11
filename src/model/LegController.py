from src.model.hardware.ServoKitSingleton import ServoKitSingleton
from src.model.hardware.Motor import Motor
from src.model.custom_types.index import LegPart

class LegController:
    """Handles control for a single leg of the robot."""
    def __init__(self, shoulder: Motor, elbow: Motor, hip: Motor, FB_is_opposited: bool, LR_is_opposited: bool):
        self.shoulder = shoulder
        self.elbow = elbow
        self.hip = hip
        self.motors = {
            LegPart.SHOULDER: shoulder,
            LegPart.ELBOW: elbow,
            LegPart.HIP: hip,
        }
        self.FB_is_opposited = FB_is_opposited
        self.LR_is_opposited = LR_is_opposited

        self.kit = ServoKitSingleton.get_instance()

    def _get_servo_angle_by_id(self, motor_id: Motor):
        angle = self.kit.servo[motor_id].angle
        if angle is None:
            raise ValueError(f"[LegController] Servo {motor_id.name} angle is disabled")
        adjusted_angle = angle if self.FB_is_opposited else 180 - angle
        return adjusted_angle

    def get_angle(self, part: LegPart):
        motor_id = self.motors[part]
        angle = self._get_servo_angle_by_id(motor_id)
        if part == LegPart.HIP:
            angle = angle if self.LR_is_opposited else 180 - angle
        return angle

    def set_angle(self, part: LegPart, degrees: float):
        motor_id = self.motors[part]

        if degrees > 180.0:
            degrees = 180.0
            print(f"Setting {motor_id.name} to over 180 degrees -> adjusted to 180")
        elif degrees < 0.0:
            degrees = 0.0
            print(f"Setting {motor_id.name} to under 0 degrees -> adjusted to 0")

        # Adjust the angle based on whether the leg is opposited
        if part == LegPart.HIP:
            adjusted_degrees = degrees if self.LR_is_opposited else 180.0 - degrees
        else:
            adjusted_degrees = degrees if self.FB_is_opposited else 180.0 - degrees

        # [TODO]: Currently only take `int` using setter, `float` as input is necessary
        self.kit.servo[motor_id].angle = adjusted_degrees


    def set_shoulder_angle(self, degrees: float):
        """Sets the angle of the shoulder motor."""
        self.set_angle(LegPart.SHOULDER, degrees)

    def set_elbow_angle(self, degrees: float):
        """Sets the angle of the elbow motor."""
        self.set_angle(LegPart.ELBOW, degrees)

    def set_hip_angle(self, degrees: float):
        """Sets the angle of the hip motor."""
        self.set_angle(LegPart.HIP, degrees)

    def pose1(self):
        self.set_shoulder_angle(90)
        self.set_elbow_angle(180)
        self.set_hip_angle(90)
