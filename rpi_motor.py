import time
import board
import busio
from adafruit_pca9685 import PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50
servo_channels = {
    "servo_channel1": 1,
    "servo_channel2": 2,
    "servo_channel3": 3,
    "servo_channel4": 4,
    "servo_channel5": 5,
    "servo_channel6": 6,
    "servo_channel7": 7,
    "servo_channel8": 8,
    "servo_channel9": 9,
    "servo_channel10": 10,
    "servo_channel11": 11,
    "servo_channel12": 12,
}

def set_servo_angle(channel, angle):
    pulse_width = 500 + (angle / 180) * 2000
    pca.channels[channel].duty_cycle = int(pulse_width * 65535 / 20000)


if __name__ == '__main__':
    try:
        for servo_name, channel in servo_channels.items():
            print(f"測試 {servo_name}，設置 90 度")
            set_servo_angle(channel, 90)
            time.sleep(1)
            set_servo_angle(channel, 100)
            print(f"測試 {servo_name} finished")
            time.sleep(1)
    except KeyboardInterrupt:
        print("終止")
    finally:
        pca.deinit()