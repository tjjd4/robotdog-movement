import time
import board
import busio
import adafruit_pca9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
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
    print("pulse width: ", pulse_width)
    pca.channels[channel].duty_cycle = int(pulse_width * 65535 / 20000)
    print("duty cycle: ", pca.channels[channel].duty_cycle)

def get_servo_angle(channel):
    # 取得當前的佔空比 (0 ~ 65535)
    duty_cycle = pca.channels[channel].duty_cycle
    
    # 計算對應的脈衝寬度，根據 16-bit (65535) 的範圍轉換為 0 ~ 20000 微秒的範圍
    pulse_width = (duty_cycle / 65535) * 20000
    
    # 將脈衝寬度轉換回角度
    if pulse_width < 500:  # 低於 500 是無效值
        pulse_width = 500
    elif pulse_width > 2500:  # 超過 2500 是無效值
        pulse_width = 2500
    
    # 計算角度
    angle = (pulse_width - 500) * 180 / 2000
    
    return angle

def angle_init():
    set_servo_angle(servo_channels["servo_channel1"], 25)
    set_servo_angle(servo_channels["servo_channel2"], 60)
    set_servo_angle(servo_channels["servo_channel3"], 0)
    set_servo_angle(servo_channels["servo_channel4"], 25)
    set_servo_angle(servo_channels["servo_channel5"], 49)
    set_servo_angle(servo_channels["servo_channel6"], 25)
    set_servo_angle(servo_channels["servo_channel7"], 25)
    set_servo_angle(servo_channels["servo_channel8"], 49)
    set_servo_angle(servo_channels["servo_channel9"], 25)
    set_servo_angle(servo_channels["servo_channel10"], 25)
    set_servo_angle(servo_channels["servo_channel11"], 49)
    set_servo_angle(servo_channels["servo_channel12"], 25)

if __name__ == '__main__':
    try:
        print("---開始---")
        angle_init()
        time.sleep(1)
        for servo_name, channel in servo_channels.items():
            angle = get_servo_angle(channel)
            print(f"測試 {servo_name}，設置 {angle} + 20 = {angle + 20} 度")
            set_servo_angle(channel, angle + 20)
            time.sleep(1)
            print(f"測試 {servo_name} finished")
            time.sleep(1)
        print("---動作結束---")
    except KeyboardInterrupt:
        print("終止")
    finally:
        angle_init()
        pca.deinit()
        print("---結束---")