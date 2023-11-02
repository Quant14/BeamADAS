import serial
import numpy as np
import struct
import time

ser = serial.Serial('/dev/ttyS0', baudrate=115200)

# Check communication
print('Waiting for connection...')
# ser.timeout = 60
if ser.readline() == b'Pi check connection\n':
    # ser.timeout = 5
    ser.write(b'Host check connection\n')
    if ser.readline() == b'OK\n':
        print('OK')
        # ser.timeout = None

print('a')
size = struct.unpack('H', ser.read(2))[0]
print('b')
data = struct.unpack(f'{size}f', ser.read(4 * size))
print('c')

lidar_data = np.array(data).reshape((size // 3, 3))
# Do work

ser.close()