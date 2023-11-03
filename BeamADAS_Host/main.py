# This script will gather sensor information from BeamNG.tech and send it to the external controller for processing

import host

import time
import matplotlib.pyplot as plt
import numpy as np

from PIL import Image
width = 1280
height = 720

import serial
import struct
import io

adas_state = 0 # states: 0 = off, 1 = on, 2 = ready, 3 = active

# Initialize simulation
print('Initializing simulation...')
home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer = host.init()
ser = serial.Serial('COM6', baudrate=115200)
stream = io.BytesIO()
adas_state = 1

vehicle.sensors.poll('electrics', 'timer', 'state')
speed = electrics.data['wheelspeed']
second = 0

# Check connection with Pi4 to set adas_state to ready
print('Connecting to Pi4...')
# ser.timeout = 5
ser.write(b'Pi check connection\n')
if ser.readline() == b'Host check connection\n':
    ser.write(b'OK\n')
    print('OK')
    adas_state = 2
    # ser.timeout = None

    while(electrics.data['running']):
        bng.pause()

        # Update misc data
        vehicle.sensors.poll('electrics', 'timer', 'state')
        speed = electrics.data['wheelspeed']

        # Get ADAS sensors data
        if speed >= 8.333: # Speed for LiDAR
            lidar_data_readonly = lidar.stream()
            pos = np.array(vehicle.state['pos'])

            lidar_data = lidar_data_readonly.copy()[:np.where(lidar_data_readonly == 0)[0][0]]
            lidar_data = lidar_data.reshape((len(lidar_data) // 3, 3))

            transform = np.identity(4)
            transform[:3, 3] = -pos

            lidar_data = np.column_stack((lidar_data, np.ones(len(lidar_data))))
            lidar_data = np.dot(lidar_data, transform.T)[:, :3]

            lidar_data = lidar_data.reshape(-1)
            ser.write(struct.pack('H', len(lidar_data)) + b''.join(struct.pack('f', f) for f in lidar_data))

            if speed >= 11.111 and second % 3 == 0: # Speed for camera
                camera_data = camera.stream_colour(3686400)
                camera_data = np.array(camera_data).reshape(height, width, 4)
                camera_data = (0.299 * camera_data[:, :, 0] + 0.587 * camera_data[:, :, 1] + 0.114 * camera_data[:, :, 2]).astype(np.uint8)
                camera_data = Image.fromarray(camera_data, 'L')
                # Method 1 - 40.602 seconds
                t1 = time.time()
                camera_data.save('sh_mem.png', 'PNG')
                img = open('sh_mem.png', 'rb').read()
                ser.write(len(img).to_bytes(4) + img)
                t1 = time.time() - t1
                # Method 2 - 79.903 seconds
                t2 = time.time()
                img = camera_data.tobytes()
                ser.write(len(img).to_bytes(4) + img)
                t2 = time.time() - t2
                # Method 3 - 40.671 seconds
                t3 = time.time()
                stream.seek(0)
                stream.truncate(0)
                camera_data.save(stream, 'PNG')
                img = stream.getvalue()
                ser.write(len(img).to_bytes(4) + img)
                t3 = time.time() - t3
                print(t1)
                print(t2)
                print(t3)
                break

        elif speed <= 3.333 and second % 3 == 0: # Parking speed
            park_data = [uss_f.stream(), uss_fl.stream(), uss_fr.stream(), 
                        uss_r.stream(), uss_rl.stream(), uss_rr.stream()]

        # Blind spot detection
        blind_data = [uss_left.stream(), uss_right.stream()]

        bng.resume()
        second += 1

        # Wait and read data from serial port

        # Act on the simulation
else:
    print('Connection error')

# Free resources
print('Exiting...')
stream.close()
ser.close()
host.destroy(home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer)