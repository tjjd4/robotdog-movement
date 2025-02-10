import time
from queue import LifoQueue

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