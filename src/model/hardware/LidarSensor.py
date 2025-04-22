from pyrplidar import PyRPlidar
import logging
import time

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LidarSensor:
    def __init__(self, port="/dev/ttyUSB0", baudrate=460800, use_express=False, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.use_express = use_express
        self.timeout = timeout
        self.lidar = PyRPlidar()
        self.connected = False
        self.scan_generator = None

    def connect(self, max_retries=3):
        """Connect to the lidar sensor with retry logic"""
        for attempt in range(max_retries):
            try:
                # Disconnect first to ensure clean state
                if self.connected:
                    self.disconnect()

                # Connect with a small delay
                time.sleep(0.1)
                self.lidar.connect(self.port, self.baudrate, timeout=self.timeout)
                
                # Set motor speed
                self.lidar.set_motor_pwm(500)
                time.sleep(2)  # Wait for motor to spin up
                
                # Get scan mode and initialize
                mode = self.lidar.get_scan_mode_typical()
                if self.use_express:
                    self.scan_generator = self.lidar.start_scan_express(mode)
                else:
                    self.scan_generator = self.lidar.start_scan()
                
                # Wait a moment for initialization
                time.sleep(0.1)
                
                self.connected = True
                logger.info(f"[LidarSensor] Connected successfully after {attempt + 1} attempts")
                return True
            except Exception as e:
                logger.error(f"[LidarSensor] Connection attempt {attempt + 1}/{max_retries} failed: {str(e)}")
                if attempt < max_retries - 1:
                    logger.info(f"[LidarSensor] Retrying in 1 second...")
                    time.sleep(1)
        return False

    def read_one(self):
        """Read a single measurement from the lidar"""
        if not self.connected:
            logger.warning("[LidarSensor] Not connected")
            return None

        try:
            # Get the next measurement from the generator
            for measurement in self.scan_generator():
                if measurement:
                    return measurement.quality, measurement.angle, measurement.distance
            return None
        except StopIteration:
            logger.warning("[LidarSensor] Scan generator stopped")
            return None
        except Exception as e:
            logger.error(f"[LidarSensor] Read error: {str(e)}")
            return None

    def disconnect(self):
        """Disconnect from the lidar sensor"""
        try:
            if self.connected:
                try:
                    # Stop motor
                    self.lidar.set_motor_pwm(0)
                    
                    # Add a small delay before disconnecting
                    time.sleep(0.1)
                    self.lidar.disconnect()
                    self.connected = False
                    logger.info("[LidarSensor] Disconnected successfully")
                except Exception as e:
                    logger.error(f"[LidarSensor] Error disconnecting: {str(e)}")
        except Exception as e:
            logger.error(f"[LidarSensor] Disconnection failed: {str(e)}")
            raise

    def __del__(self):
        """Clean up resources when the object is destroyed"""
        try:
            self.disconnect()
        except Exception as e:
            logger.error(f"[LidarSensor] Error in destructor: {str(e)}")
