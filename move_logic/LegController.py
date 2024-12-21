from hardware.ServoKitSingleton import ServoKitSingleton
from hardware.Motor import Motor
from types.leg import LegPart

class LegController:
    """Handles control for a single leg of the robot."""
    def __init__(self, shoulder: Motor, elbow: Motor, hip: Motor, is_opposited: bool):
        self.shoulder = shoulder
        self.elbow = elbow
        self.hip = hip
        self.motors = {
            LegPart.SHOULDER: shoulder,
            LegPart.ELBOW: elbow,
            LegPart.HIP: hip,
        }
        self.is_opposited = is_opposited

        self.kit = ServoKitSingleton.get_instance()

    def get_angle(self, motor_id: Motor):
        return self.kit.servo[motor_id].angle

    def set_angle(self, motor_id: Motor, degrees: int):
        if (degrees > 180):
            degrees = 180
            print(f"Setting {motor_id.name} to over 180 degree -> adjust to 180")
        elif (degrees < 0):
            degrees = 0
            print(f"Setting {motor_id.name} to under 0 degree -> adjust to 0")
        self.kit.servo[motor_id].angle =  degrees if self.is_opposited else 180 - degrees
        print(f"Set motor {motor_id.name} to angle {degrees} degrees")

    def get_angle_by_module(self, part: LegPart):
        motor_id = self.motors[part]
        return self.kit.servo[motor_id].angle

    def set_angle_by_module(self, part: LegPart, degrees: int):
        motor_id = self.motors[part]

        if degrees > 180:
            degrees = 180
            print(f"Setting {motor_id.name} to over 180 degrees -> adjusted to 180")
        elif degrees < 0:
            degrees = 0
            print(f"Setting {motor_id.name} to under 0 degrees -> adjusted to 0")

        # Adjust the angle based on whether the leg is opposited
        adjusted_degrees = degrees if self.is_opposited else 180 - degrees
        self.kit.servo[motor_id].angle = adjusted_degrees


    def set_shoulder_angle(self, degrees: int):
        """Sets the angle of the shoulder motor."""
        self.set_angle(self.shoulder, degrees)
        print(f"Set shoulder motor ({self.shoulder.name}) to angle {degrees} degrees")

    def set_elbow_angle(self, degrees: int):
        """Sets the angle of the elbow motor."""
        self.set_angle(self.elbow, degrees)
        print(f"Set elbow motor ({self.elbow.name}) to angle {degrees} degrees")

    def set_hip_angle(self, degrees: int):
        """Sets the angle of the hip motor."""
        self.set_angle(self.hip, degrees)
        print(f"Set hip motor ({self.hip.name}) to angle {degrees} degrees")