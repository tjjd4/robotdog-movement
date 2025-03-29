import threading
import numpy as np
import open3d as o3d
import time

class LidarController:
    def __init__(self):
        self.latest_point_cloud = np.empty((0, 3))
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

    def start(self, data_source_func, interval=0.1):
        self.running = True
        self.thread = threading.Thread(target=self._lidar_data_update_loop, args=(data_source_func, interval), daemon=True)
        self.thread.start()
        print("[LIDAR] Started real-time LIDAR data thread.")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
            print("[LIDAR] LIDAR thread stopped.")

    def _lidar_data_update_loop(self, data_source_func, interval):
        while self.running:
            lidar_data = data_source_func()
            point_cloud = self.convert_lidar_to_point_cloud(lidar_data)
            with self.lock:
                self.latest_point_cloud = point_cloud
            time.sleep(interval)

    def convert_lidar_to_point_cloud(self, lidar_data):
        angles = np.radians(np.array([d[0] for d in lidar_data]))
        distances = np.array([d[1] for d in lidar_data])
        heights = np.array([d[2] for d in lidar_data])
        qualities = np.array([d[3] for d in lidar_data])
        mask = (qualities > 0) & (distances > 0)
        x = distances[mask] * np.cos(angles[mask])
        y = distances[mask] * np.sin(angles[mask])
        z = heights[mask]
        return np.column_stack((x, y, z))

    def get_latest_point_cloud(self):
        with self.lock:
            return self.latest_point_cloud.copy()

    def visualize_latest_point_cloud(self):
        with self.lock:
            if self.latest_point_cloud.size == 0:
                print("No LIDAR points to display.")
                return
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.latest_point_cloud)
            o3d.visualization.draw_geometries([pcd, self.create_coordinate_axes()])

    def create_coordinate_axes(self, size=3.0):
        return o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)

    def save_latest_point_cloud(self, filename="lidar_output.pcd"):
        with self.lock:
            if self.latest_point_cloud is not None and self.latest_point_cloud.size > 0:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(self.latest_point_cloud)
                o3d.io.write_point_cloud(filename, pcd)
                print(f"[LIDAR] Saved point cloud to {filename}")
            else:
                print("[LIDAR] No point cloud to save.")
