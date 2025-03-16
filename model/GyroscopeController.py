import time
from threading import Thread

from .hardware.GyroKitSingleton import GyroKitSingleton
from .custom_types.index import GyroData
from utils.GyroQueue import GyroQueue

class GyroscopeController:
    def __init__(self):
        self.gyro_kit = GyroKitSingleton.get_instance()
        self.packet_size = self.gyro_kit.DMP_get_FIFO_packet_size()

    def read_gyro_data(self):
        FIFO_buffer = [0] * 42

        while True:
            if self.gyro_kit.isreadyFIFO(self.packet_size):  # Check if FIFO data are ready to use...
                FIFO_buffer = self.gyro_kit.get_FIFO_bytes(self.packet_size)  # get all the DMP data here
                
                q = self.gyro_kit.DMP_get_quaternion_int16(FIFO_buffer)
                roll_pitch_yaw = self.gyro_kit.DMP_get_euler_roll_pitch_yaw(q)
                roll = roll_pitch_yaw.x
                pitch = roll_pitch_yaw.y
                yaw = roll_pitch_yaw.z
                return GyroData(roll=roll, pitch=pitch, yaw=yaw)


if __name__ == "__main__":
    gyro_controller = GyroscopeController()
    try:
        while True:
            data: GyroData = gyro_controller.read_gyro_data()
            print(f"Roll: {data.roll:.2f}, Pitch: {data.pitch:.2f}, Yaw: {data.yaw:.2f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("LOG: Stop reading gyro data")