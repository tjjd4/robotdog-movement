import time
from queue import LifoQueue
from model.GyroscopeController import GyroscopeController
from model.custom_types.index import GyroData

if __name__ == "__main__":
    gyro_controller = GyroscopeController()

    try:
        while True:
            data: GyroData = gyro_controller.read_gyro_data()
            print(f"Roll: {data.roll:.2f}, Pitch: {data.pitch:.2f}, Yaw: {data.yaw:.2f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("LOG: Stop reading gyro data")