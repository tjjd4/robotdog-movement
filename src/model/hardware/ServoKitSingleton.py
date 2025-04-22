from adafruit_servokit import ServoKit

class ServoKitSingleton:
    """Singleton manager for ServoKit."""
    _instance = None

    @staticmethod
    def get_instance():
        if ServoKitSingleton._instance is None:
            ServoKitSingleton._instance = ServoKit(channels=16)
            for i in range(1, 13):
                ServoKitSingleton._instance.servo[i].set_pulse_width_range(500,2500)
            print("ServoKit instance created.")
        return ServoKitSingleton._instance


# Usage example:
# servo_kit_singleton = ServoKitSingleton()
# kit = servo_kit_singleton.get_instance()
# kit.servo[0].angle = 90