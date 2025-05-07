import logging
import time
from typing import Optional
import threading

from src.model.hardware.GyroKitSingleton import GyroKitSingleton
from src.model.custom_types.index import GyroData

logger = logging.getLogger(__name__)

class GyroSensor:
    def __init__(self):
        self.gyro_kit = GyroKitSingleton.get_instance()
        self.packet_size = self.gyro_kit.DMP_get_FIFO_packet_size()
        self.connected = False
        self._fifo_buffer = [0] * 42
        self._lock = threading.Lock()

    def connect(self, max_retries: int = 3) -> bool:
        """Initialize and connect to the gyroscope sensor with retry logic"""
        for attempt in range(max_retries):
            try:
                # Initialize FIFO buffer
                self._fifo_buffer = [0] * 42
                
                # Check if sensor is ready
                if self.gyro_kit.isreadyFIFO(self.packet_size):
                    with self._lock:
                        self.connected = True
                    logger.info(f"[GyroscopeSensor] Connected successfully after {attempt + 1} attempts")
                    return True
                else:
                    raise RuntimeError("Gyroscope sensor not ready")
            except Exception as e:
                logger.error(f"[GyroscopeSensor] Connection attempt {attempt + 1}/{max_retries} failed: {str(e)}")
                if attempt < max_retries - 1:
                    logger.info(f"[GyroscopeSensor] Retrying in 1 second...")
                    time.sleep(1)
        return False

    def read(self) -> Optional[GyroData]:
        """Read gyroscope data (roll, pitch, yaw)"""
        with self._lock:
            if not self.connected:
                logger.warning("[GyroscopeSensor] Not connected")
                return None

        try:
            if self.gyro_kit.isreadyFIFO(self.packet_size):
                self._fifo_buffer = self.gyro_kit.get_FIFO_bytes(self.packet_size)
                
                q = self.gyro_kit.DMP_get_quaternion_int16(self._fifo_buffer)
                roll_pitch_yaw = self.gyro_kit.DMP_get_euler_roll_pitch_yaw(q)
                
                return GyroData(
                    roll=roll_pitch_yaw.x,
                    pitch=roll_pitch_yaw.y,
                    yaw=roll_pitch_yaw.z
                )
            return None
        except Exception as e:
            logger.error(f"[GyroscopeSensor] Read error: {str(e)}")
            return None

    def disconnect(self) -> None:
        """Disconnect from the gyroscope sensor"""
        try:
            with self._lock:
                if self.connected:
                    self.connected = False
            logger.info("[GyroscopeSensor] Disconnected successfully")
        except Exception as e:
            logger.error(f"[GyroscopeSensor] Disconnection failed: {str(e)}")
            raise

    def __del__(self):
        """Clean up resources when the object is destroyed"""
        try:
            self.disconnect()
        except Exception as e:
            logger.error(f"[GyroscopeSensor] Error in destructor: {str(e)}")