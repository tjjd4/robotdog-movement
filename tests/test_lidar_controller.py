from model.LidarController import LidarController
import time

if __name__ == "__main__":
    controller = LidarController()
    controller.start()

    try:
        while True:
            scan = controller.get_distances()
            print(f"[掃描資料] 點數: {len(scan)}")
            for angle in sorted(scan.keys())[:10]:  # 顯示前 10 筆
                print(f"角度: {angle:.2f}° ➝ 距離: {scan[angle]:.2f} mm")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("中止")
    finally:
        controller.stop()
