from model.hardware.LidarSensor import LidarSensor
import threading

class LidarController:
    def __init__(self, sensor: LidarSensor=None):
        self.sensor = LidarSensor() if sensor is None else sensor
        self.running = False
        self.thread = None
        self.latest_scan = [None] * 360  # index 0~359 -> 最新距離資料
        self.lock = threading.Lock()

    def start(self):
        self.sensor.connect()
        self.running = True
        self.thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.thread.start()
        print("[LidarController] Start scanning")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        self.sensor.disconnect()
        print("[LidarController] Stop scanning")

    def _scan_loop(self):
        while self.running:
            try:
                quality, angle, distance = self.sensor.read_one()
                if quality == 0:
                    continue
                angle_idx = int(angle) % 360
                with self.lock:
                    self.latest_scan[angle_idx] = distance
            except Exception as e:
                print("[LidarController] Error:", e)

    def get_latest_scan(self):
        with self.lock:
            return list(self.latest_scan)  # copy to avoid race condition
