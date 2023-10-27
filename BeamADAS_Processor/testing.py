import serial

ser = serial.Serial('/dev/ttyS0', baudrate=115200)

# Check communication
ser.timeout = 60
if ser.readline() == b'Pi check connection\n':
    ser.timeout = 1
    ser.write(b'Host check connection\n')
    if ser.readline() == b'OK\n':
        print('OK')
        ser.timeout = None

# Do work

ser.close()
