from pyrplidar import PyRPlidar
import time

def simple_express_scan():

    lidar = PyRPlidar()
    lidar.connect(port="/dev/ttyUSB0", baudrate=460800, timeout=3)
                  
    lidar.set_motor_pwm(500)
    time.sleep(2)
    
    # mode = lidar.get_scan_mode_typical()
    # scan_generator = lidar.start_scan_express(mode)
    scan_generator = lidar.start_scan()
    
    for count, scan in enumerate(scan_generator()):
        print(count, scan)
        if count == 20: break

    lidar.stop()
    lidar.set_motor_pwm(0)

    
    lidar.disconnect()


if __name__ == "__main__":
    simple_express_scan()


"""
This code below is used to test the Lidar without using any package.
It is used to verify the raw data from the Lidar.
"""


# import serial
# import time

# PORT = "/dev/ttyUSB0"
# BAUDRATE = 460800

# def parse_scan_data(data):
#     if len(data) < 5:
#         return None
#     quality = data[0] >> 2
#     angle = ((data[1] | (data[2] << 8)) >> 1) / 64.0
#     distance = (data[3] | (data[4] << 8)) / 4.0
#     return quality, angle, distance

# with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
#     ser.write(b'\xA5\x20')  # 啟動掃描模式
#     time.sleep(0.1)

#     while True:
#         raw = ser.read(5)
#         if len(raw) == 5:
#             result = parse_scan_data(raw)
#             if result:
#                 q, a, d = result
#                 print(f"品質: {q}, 角度: {a:.2f}, 距離: {d:.2f} mm")
