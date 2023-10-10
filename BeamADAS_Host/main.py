# This script will gather sensor information from BeamNG.tech and send it to the external controller for processing

import host

import time
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
width = 1280
height = 720

adas_state = 0 # States - 0 = off, 1 = on, 2 = ready, 3 = active

# Initialize simulation
home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer = host.init()
adas_state = 1

vehicle.sensors.poll('electrics', 'timer', 'state')
speed = electrics.data['wheelspeed']
second = 0

# Check connection with Pi4 to set adas_state to ready

while(electrics.data['running']):
    bng.pause()

    # Update misc data
    vehicle.sensors.poll('electrics', 'timer', 'state')
    speed = electrics.data['wheelspeed']
    
    # Get ADAS sensors data

    if speed >= 8.333: # Speed for LiDAR
        lidar_data_readonly = lidar.stream()
        lidar_data = lidar_data_readonly.copy()
        pos = vehicle.state['pos']

        # Not needed as serial port would transfer the same number of bytes
        # for i in range(0, len(lidar_data)):
        #     if lidar_data[i] == 0:
        #         break
        #     lidar_data[i] = round(lidar_data[i] - pos[i % 3], 3)

        if speed >= 11.111 and second % 3 == 0: # Speed for camera
            camera_data = camera.stream_colour(3686400)
            camera_data = np.array(camera_data).reshape(height, width, 4)
            camera_data = (0.299 * camera_data[:, :, 0] + 0.587 * camera_data[:, :, 1] + 0.114 * camera_data[:, :, 2]).astype(np.uint8)
            Image.fromarray(camera_data, 'L').save("sh_mem.png", "PNG") # Replace with send over serial port
    
    elif speed <= 3.333 and second % 3 == 0: # Parking speed
        park_data = [uss_f.stream(), uss_fl.stream(), uss_fr.stream(), 
                    uss_r.stream(), uss_rl.stream(), uss_rr.stream()]

    # Blind spot detection
    blind_data = [uss_left.stream(), uss_right.stream()]

    bng.resume()

    # Wait and read data from serial port

    # Act on the simulation
    second += 1
    vehicle.control(throttle=(second % 100) / 100)
# Free resources
host.destroy(home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer)