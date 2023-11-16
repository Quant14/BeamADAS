# This script will gather sensor information from BeamNG.tech and send it to the external controller for processing

import host

import time
import matplotlib.pyplot as plt
import numpy as np

from PIL import Image
width = 1280
height = 720

import paramiko
pi_ip = "fe80::4464:3bc8:5788:fbfb"
user = "quant"
passwd = "adas"

adas_state = 0 # states: 0 = off, 1 = on, 2 = ready, 3 = active

# Initialize simulation
print('Initializing simulation...')
# home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer = host.init('sp1_no_traffic', False)
adas_state = 1

# vehicle.sensors.poll('electrics', 'timer', 'state')
# speed = electrics.data['wheelspeed']
second = 0

# Check connection with Pi4 to set adas_state to ready
print('Connecting to Pi4...')
lidar_data = np.genfromtxt('sp1/sample1/lidar/pc0.txt')
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(pi_ip, username=user, password=passwd)
stdin, stdout, stderr = ssh.exec_command("echo Initialize")
stdin.write(lidar_data.tobytes())
stdin.channel.shutdown_write()
ssh.close()
exit()
print('a')
while(electrics.data['running']):
    bng.pause()

    # Update misc data
    vehicle.sensors.poll('electrics', 'timer', 'state')
    speed = electrics.data['wheelspeed']

    # Get ADAS sensors data
    if speed >= 8.333: # Speed for LiDAR
        lidar_data_readonly = lidar.stream()
        pos = np.array([vehicle.state['pos'][0], vehicle.state['pos'][1] - 2.25, vehicle.state['pos'][2] + 0.6])
        direction = vehicle.state['dir']

        lidar_data = lidar_data_readonly.copy()[:np.where(lidar_data_readonly == 0)[0][0]]
        lidar_data = lidar_data.reshape((len(lidar_data) // 3, 3))

        transform = np.identity(4)
        transform[:3, 3] = -pos

        lidar_data = np.column_stack((lidar_data, np.ones(len(lidar_data))))
        lidar_data = np.dot(lidar_data, transform.T)[:, :3]

        vectors = lidar_data - np.array([0, 0, 0])
        lidar_data = lidar_data[np.dot(vectors, direction) >= 0]

        # lidar_data = lidar_data.reshape(-1)
        stdin.write(lidar_data.tobytes())
        # write

        if speed >= 11.111 and second % 3 == 0: # Speed for camera
            camera_data = camera.stream_colour(3686400)
            camera_data = np.array(camera_data).reshape(height, width, 4)
            camera_data = (0.299 * camera_data[:, :, 0] + 0.587 * camera_data[:, :, 1] + 0.114 * camera_data[:, :, 2]).astype(np.uint8)
            camera_data = Image.fromarray(camera_data, 'L')

            camera_data.save('sh_mem.png', 'PNG')
            img = open('sh_mem.png', 'rb').read()
            # write

    elif speed <= 3.333 and second % 3 == 0: # Parking speed
        park_data = [uss_f.stream(), uss_fl.stream(), uss_fr.stream(), 
                    uss_r.stream(), uss_rl.stream(), uss_rr.stream()]

    # Blind spot detection
    blind_data = [uss_left.stream(), uss_right.stream()]

    bng.resume()
    second += 1

        # Wait and read data from serial port

        # Act on the simulation

# Free resources
print('Exiting...')
ssh.close()
host.destroy(home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer)