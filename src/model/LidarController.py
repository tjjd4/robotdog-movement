import threading
import numpy as np
import matplotlib.pyplot as plt
from io import BytesIO

from src.model.hardware.LidarSensor import LidarSensor

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
        
    def generate_frames(self):
        """產生繪製後的 LIDAR 點雲圖 JPEG 串流"""
        while self.running:
            scan = self.get_latest_scan()
            points = [
                (
                    dist * np.cos(np.radians(angle)),
                    dist * np.sin(np.radians(angle))
                )
                for angle, dist in enumerate(scan)
                if dist is not None
            ]

            fig, ax = plt.subplots(figsize=(6, 6))
            ax.set_xlim(-1000, 1000)
            ax.set_ylim(-1000, 1000)
            ax.set_aspect('equal')
            ax.set_facecolor('black')
            ax.axis('off')  # 移除坐標軸

            if points:
                xs, ys = zip(*points)
                ax.scatter(xs, ys, s=2, c='lime')

            buf = BytesIO()
            plt.savefig(buf, format='jpeg', dpi=100, bbox_inches='tight', pad_inches=0)
            plt.close(fig)
            buf.seek(0)

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buf.read() + b'\r\n')

    def is_running(self):
        return self.running
