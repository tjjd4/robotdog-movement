from model.LidarController import LidarController
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

def polar_to_cartesian(scan_dict):
    """將掃描結果轉換成 (x, y) 點雲"""
    points = [
        (
            dist * np.cos(np.radians(angle)),
            dist * np.sin(np.radians(angle))
        )
        for angle, dist in scan_dict.items()
        if dist is not None
    ]

    return np.array(points)

if __name__ == "__main__":
    controller = LidarController()
    controller.start()
    run_time = 0
    try:
        print("Visualize the map")
        while True:
            fig, ax = plt.subplots()
            ax.set_xlim(-1000, 1000)
            ax.set_ylim(-1000, 1000)
            ax.set_aspect('equal')
            ax.set_title("LIDAR 2D Scan Visualization")
            ax.grid(True)

            sc = ax.scatter([], [], s=2, c='lime')

            def update(frame):
                scan = controller.get_latest_scan()
                points = polar_to_cartesian(scan)
                if points.size > 0:
                    sc.set_offsets(points)
                return sc,

            ani = animation.FuncAnimation(fig, update, interval=200)
            plt.show()
        
    except KeyboardInterrupt:
        print("中止")
    finally:
        controller.stop()
