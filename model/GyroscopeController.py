import time
from threading import Thread
from queue import LifoQueue
from .hardware.GyroKitSingleton import GyroKitSingleton
from .types.types import GyroData
class GyroscopeController:
    def __init__(self, gyro_queue: LifoQueue):
        self.gyro_kit = GyroKitSingleton.get_instance()
        self.queue = gyro_queue
        self.running = False
        self.gyro_thread = Thread()

    def start(self):
        if not self.gyro_thread.is_alive():
                print("Start gyroscope detecting thread...")
                self.running = True
                self.gyro_thread = Thread(target=self.__detect)
                self.gyro_thread.start()
        else:
            print("Already detecting!")

    def __detect(self):
        packet_size = self.gyro_kit.DMP_get_FIFO_packet_size()
        FIFO_buffer = [0] * 64

        while self.running:  # infinite loop
            if self.gyro_kit.isreadyFIFO(packet_size):  # Check if FIFO data are ready to use...
                FIFO_buffer = self.gyro_kit.get_FIFO_bytes(packet_size)  # get all the DMP data here
                
                q = self.gyro_kit.DMP_get_quaternion_int16(FIFO_buffer)
                roll, pitch, yaw = self.gyro_kit.DMP_get_euler_roll_pitch_yaw(q)
                self.queue.put(GyroData(roll=roll, pitch=pitch, yaw=yaw))
                
                print('roll: ' + str(roll))
                print('pitch: ' + str(pitch))
                print('yaw: ' + str(yaw))
                print('\n')
            time.sleep(0.05)
        
    def stop(self):
        if self.gyro_thread.is_alive():
                print("Kill gyroscope detecting thread...")
                self.running = False
                self.gyro_thread.join()
        else:
            print("detection not activated!")

    def is_running(self):
         return self.running


if __name__ == "__main__":
    q = LifoQueue()
    gyro_controller = GyroscopeController(q)
    gyro_controller.start()

    try:
        while True:
            if not q.empty():
                data: GyroData = q.get()
                print(f"Roll: {data.roll:.2f}, Pitch: {data.pitch:.2f}, Yaw: {data.yaw:.2f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        gyro_controller.stop()