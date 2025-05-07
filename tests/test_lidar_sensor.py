import time
import logging

from src.model.hardware.LidarSensor import LidarSensor

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

if __name__ == "__main__":
    try:
        sensor = LidarSensor()
        if not sensor.connect():
            logger.error("[LidarSensorTest] Failed to connect to lidar sensor")
            exit(1)

        print("[LidarSensorTest] Start reading...")
        start_time = time.time()
        duration = 5  # testing duration in seconds
        
        while time.time() - start_time < duration:
            try:
                result = sensor.read_one()
                if result:
                    quality, angle, distance = result
                    print(f"品质: {quality}, 角度: {angle}, 距离: {distance} mm")
                else:
                    print("[LidarSensorTest] Miss")
            except Exception as e:
                logger.error(f"[LidarSensorTest] Error: {str(e)}")

        print(f"[LidarSensorTest] Finished reading after {duration} seconds")

    except Exception as e:
        logger.error(f"[LidarSensorTest] Main error: {str(e)}")
    finally:
        try:
            sensor.disconnect()
            print("[LidarSensorTest] Disconnected")
        except Exception as e:
            logger.error(f"[LidarSensorTest] Error disconnecting: {str(e)}")
