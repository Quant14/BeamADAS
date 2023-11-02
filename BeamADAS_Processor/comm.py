import serial
import struct

ser = serial.Serial('/dev/ttyS0', baudrate=115200)

# Check communication
print('Waiting for connection...')
ser.timeout = 60
if ser.readline() == b'Pi check connection\n':
    ser.timeout = 1
    ser.write(b'Host check connection\n')
    if ser.readline() == b'OK\n':
        print('OK')
        ser.timeout = None

size = struct.unpack('H', ser.read(2))[0]
data = struct.unpack(f'{size}f', ser.read(4 * size))
# Do work

ser.close()
