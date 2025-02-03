import time
from mpu6050.MPU6050 import MPU6050

i2c_bus = 0
device_address = 0x68
freq_divider = 0x10

# Make an MPU6050
mpu = MPU6050(i2c_bus, device_address)

# Initiate your DMP
mpu.dmp_initialize()
mpu.set_DMP_enabled(True)

packet_size = mpu.DMP_get_FIFO_packet_size()
FIFO_buffer = [0]*64

while True: # infinite loop
    if mpu.isreadyFIFO(packet_size): # Check if FIFO data are ready to use...
        
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size) # get all the DMP data here
        
        q = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(q)
        roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(q)
        
        print('roll: ' + str(roll_pitch_yaw.x))
        print('pitch: ' + str(roll_pitch_yaw.y))
        print('yaw: ' + str(roll_pitch_yaw.z))
        print('\n')
    time.sleep(0.05)