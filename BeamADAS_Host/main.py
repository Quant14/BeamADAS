# This script will gather sensor information from BeamNG.tech and send it to the external controller for processing

import host

import time
# import matplotlib.pyplot as plt
# import numpy as np
# from decimal import Decimal, ROUND_HALF_UP

# Initialize simulation
home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer = host.init()

vehicle.sensors.poll('electrics')
speed = electrics.data['wheelspeed']
# timestamp = 0.0

while(electrics.data['running']):
    if speed >= 8.333: # Speed for LiDAR
        # vehicle.sensors.poll('state')
        lidar_data_readonly = lidar.poll()['pointCloud']
        # lidar_data = lidar_data_readonly.copy()
        # pos = vehicle.state['pos']
        # i = 0

        # for i in range(0, len(lidar_data)):
        #     if lidar_data[i] == 0:
        #         break
        #     lidar_data[i] = round(lidar_data[i] - pos[i % 3], 3)
        if speed >= 11.111: # Speed for camera
            # timestamp = timer.data['time']
            camera_data = camera.poll()['colour'].convert('L')
    # elif speed <= 3.333 and host.is_elapsed(timer, vehicle, timestamp): # Parking speed
    #     park_data = [round(uss_f.poll()['distance'], 3), round(uss_fl.poll()['distance'], 3), round(uss_fr.poll()['distance'], 3), 
    #                 round(uss_r.poll()['distance'], 3), round(uss_rl.poll()['distance'], 3), round(uss_rr.poll()['distance'], 3)]
    #     timestamp = timer.data['time']

    # Blind spot detection
    # blind_data = [round(uss_left.poll()['distance']), round(uss_right.poll()['distance'])]

    vehicle.sensors.poll('timer')
    print(timer.data['time'])

    # Update misc data
    vehicle.sensors.poll('electrics')
    speed = electrics.data['wheelspeed']

# Free resources
host.destroy(home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer)