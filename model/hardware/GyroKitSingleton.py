import time
from mpu6050.MPU6050 import MPU6050

class GyroKitSingleton:
    """Singleton manager for MPU6050."""
    _instance = None

    @staticmethod
    def get_instance(i2c_bus=0, device_address=0x68, freq_divider=0x10):
        if GyroKitSingleton._instance is None:
            GyroKitSingleton._instance = MPU6050(i2c_bus, device_address, freq_divider)
            GyroKitSingleton._instance.dmp_initialize()
            GyroKitSingleton._instance.set_DMP_enabled(True)
            print("MPU6050 instance created.")
        return GyroKitSingleton._instance