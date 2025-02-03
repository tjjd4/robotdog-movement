import smbus

address = 0x68  # MPU6050 的 I2C 地址
bus = smbus.SMBus(0)

try:
    who_am_i = bus.read_byte_data(address, 0x75)  # 讀取 WHO_AM_I 註冊表
    print(f"MPU6050 WHO_AM_I: {who_am_i:#x}")  # 預期輸出 0x68
except OSError as e:
    print(f"I2C Error: {e}")
