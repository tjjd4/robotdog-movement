import time
from queue import LifoQueue
from model.GyroscopeController import GyroscopeController
from model.custom_types.index import GyroData

if __name__ == "__main__":
    gyro_controller = GyroscopeController()

    try:
        while True:
            timestamp = time.perf_counter()
            data: GyroData = gyro_controller.read_gyro_data()
            print("cost: ", time.perf_counter() - timestamp)
            if data != None:
                print(f"Roll: {data.roll:.2f}, Pitch: {data.pitch:.2f}, Yaw: {data.yaw:.2f}")
            else:
                print("No data")

    except KeyboardInterrupt:
        print("LOG: Stop reading gyro data")