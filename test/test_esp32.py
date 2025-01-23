import serial

uart = serial.Serial(
    port='/dev/serial0',  # Raspberry Pi 上的默認 UART 接口
    baudrate=9600,
    timeout=1  # 1 秒超時
)

print("Raspberry Pi UART Receiver Initialized")

try:
    while True:
        if uart.in_waiting > 0:
            data = uart.readline().decode('utf-8').strip()
            print(f"Received from ESP32: {data}")

            uart.write(b"Message Received\n")
except KeyboardInterrupt:
    print("Program terminated")
finally:
    uart.close()
