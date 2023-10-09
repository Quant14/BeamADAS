# This script will gather sensor information from BeamNG.tech and send it to the external controller for processing

import host

import time
import matplotlib.pyplot as plt
import numpy as np
# from decimal import Decimal, ROUND_HALF_UP
from PIL import Image
width = 1280
height = 720

# Initialize simulation
home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer = host.init()

vehicle.sensors.poll('electrics')
speed = electrics.data['wheelspeed']
# timestamp = 0.0

while(electrics.data['running']):
    if speed >= 8.333: # Speed for LiDAR
        # vehicle.sensors.poll('state')
        lidar_data_readonly = lidar.stream()
        # lidar_data = lidar_data_readonly.copy()
        # pos = vehicle.state['pos']
        # i = 0

        # for i in range(0, len(lidar_data)):
        #     if lidar_data[i] == 0:
        #         break
        #     lidar_data[i] = round(lidar_data[i] - pos[i % 3], 3)
        if speed >= 11.111: # Speed for camera
            # timestamp = timer.data['time']
            camera_data = camera.poll_shmem_colour()
    # elif speed <= 3.333 and host.is_elapsed(timer, vehicle, timestamp): # Parking speed
    #     park_data = [round(uss_f.poll()['distance'], 3), round(uss_fl.poll()['distance'], 3), round(uss_fr.poll()['distance'], 3), 
    #                 round(uss_r.poll()['distance'], 3), round(uss_rl.poll()['distance'], 3), round(uss_rr.poll()['distance'], 3)]
    #     timestamp = timer.data['time']

    # Blind spot detection
    # Timing benchmarks
    # print(time.time())
    # blind_data = [uss_left.stream(), uss_right.stream()]
    # print(time.time())
    # lidar.stream()
    # print(time.time())
    # camera_data = camera.stream_colour(3686400)
    # print(time.time())
    # camera_data = np.array(camera_data).reshape(height, width, 4)
    # camera_data = (0.299 * camera_data[:, :, 0] + 0.587 * camera_data[:, :, 1] + 0.114 * camera_data[:, :, 2]).astype(np.uint8)
    # # Image.fromarray(camera_data, 'L').save("sh_mem.png", "PNG") # Replace with send over serial port
    # print(time.time())
    # break
    # print('1: ' + str(time.time()))
    # vehicle.sensors.poll('timer')
    vehicle.sensors.poll('timer')
    # print(timer.data['time'])
    # vehicle.control(throttle=0.75)
    # Update misc data
    vehicle.sensors.poll('electrics')
    speed = electrics.data['wheelspeed']

# Free resources
host.destroy(home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer)