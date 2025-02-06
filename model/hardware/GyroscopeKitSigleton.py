import time
from mpu6050.MPU6050 import MPU6050

class GyroscopeKitSigleton:
    """Singleton manager for MPU6050."""
    _instance = None

    @staticmethod
    def get_instance(i2c_bus=0, device_address=0x68, freq_divider=0x10):
        if GyroscopeKitSigleton._instance is None:
            GyroscopeKitSigleton._instance = MPU6050(i2c_bus, device_address, freq_divider)
            GyroscopeKitSigleton._instance.dmp_initialize()
            GyroscopeKitSigleton._instance.set_DMP_enabled(True)
            print("MPU6050 instance created.")
        return GyroscopeKitSigleton._instance