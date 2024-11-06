import numpy as np
import socket

import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from utils.config_helper import update_config_ip, update_config_port

s = None

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def create_socket_connection(start_port=5000):

    host=get_ip() # this computer's IP
    update_config_ip(host)
    port = start_port
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while port < min(start_port+1000,9999):
        try:
            s.bind((host,port))
            break
        except OSError as e:
            port+=1
    update_config_port(port)
    print(f"Connection Info - ip: {host}, port: {port}")
    return s

def controller(momentum):
    global s
    if s is None:
        s = create_socket_connection()
    try:
        s.settimeout(0.00001)
        data, addr = s.recvfrom(1024)
        if data: 
            momentum = np.frombuffer(data)
    except:
        pass
    return momentum

if __name__ == "__main__":
    # Test network commands by printing out the momentum changes
    momentum = np.asarray([0,0,1,0],dtype=np.float32)
    lm = momentum
    while True:
        momentum = controller(momentum)
        if (lm != momentum).any():
            print(momentum)
            lm=momentum