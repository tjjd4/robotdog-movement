import numpy as np
import socket
import keyboard

import configparser

config_file = "config.ini"

config = configparser.ConfigParser()
config.read(config_file)
SERVER_IP = config['controller_network'].get('ip')
SERVER_PORT = config['controller_network'].get('port')

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

momentum = np.asarray([0, 0, 1, 0], dtype=np.float32)

def update_momentum(momentum, accel=0.01, bound=4):
    
    if keyboard.is_pressed('w'):
        momentum[0] = min(momentum[0] + accel, bound)
    if keyboard.is_pressed('s'):
        momentum[0] = max(momentum[0] - accel, -bound)
    if keyboard.is_pressed('a'):
        momentum[1] = max(momentum[1] - accel, -bound)
    if keyboard.is_pressed('d'):
        momentum[1] = min(momentum[1] + accel, bound)
    return momentum

def send_momentum(momentum):
    """
    將 momentum 向量通過 UDP 發送到伺服端。
    """
    client_socket.sendto(momentum.tobytes(), (SERVER_IP, SERVER_PORT))

if __name__ == "__main__":
    print("按 'w', 'a', 's', 'd' 來控制機器人的方向")
    print("按 'q' 鍵退出")

    try:
        while True:
            # 更新 momentum 根據按鍵輸入
            momentum = update_momentum(momentum)
            
            # 發送 momentum 到伺服端
            send_momentum(momentum)
            
            # 檢查是否按下 'q' 鍵退出
            if keyboard.is_pressed('q'):
                print("退出程式")
                break
    except KeyboardInterrupt:
        print("手動中斷程式")
    finally:
        client_socket.close()