import time
from typing import Optional

from src.model.hardware.GyroSensor import GyroSensor
from src.model.custom_types.index import GyroData

class GyroController:
    def __init__(self):
        self.sensor = GyroSensor()
        self.connected = False

    def connect(self) -> bool:
        """Connect to the gyroscope sensor"""
        self.connected = self.sensor.connect()
        return self.connected

    def read_gyro_data(self) -> Optional[GyroData]:
        """Read gyroscope data (roll, pitch, yaw)"""
        if not self.connected:
            return None
        
        return self.sensor.read()

    def disconnect(self) -> None:
        """Disconnect from the gyroscope sensor"""
        if self.connected:
            self.sensor.disconnect()
            self.connected = False

if __name__ == "__main__":
    gyro_controller = GyroController()
    try:
        if not gyro_controller.connect():
            print("Failed to connect to gyroscope")
            exit(1)

        while True:
            data: GyroData = gyro_controller.read_gyro_data()
            if data:
                print(f"Roll: {data.roll:.2f}, Pitch: {data.pitch:.2f}, Yaw: {data.yaw:.2f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nLOG: Stop reading gyro data")
    finally:
        gyro_controller.disconnect()
