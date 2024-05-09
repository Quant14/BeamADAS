# This script will gather sensor information from BeamNG.tech and send it to the external controller for processing

import host

import cv2
import sys
import os
import time
import struct
import matplotlib.pyplot as plt
import numpy as np
from decimal import Decimal, ROUND_HALF_UP
from PIL import Image
width = 1280
height = 720

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, Lidar, Ultrasonic, Electrics, Timer
from beamngpy.tools import OpenDriveExporter

# dirs = ['sp1', 'sp1_no_traffic', 'sp2', 'sp2_no_traffic']
# dirs = ['sp2_no_traffic']

home, bng, scenario, vehicle, camera, lidar, uss_fl1, uss_fl2, uss_fr1, uss_fr2, uss_rl1, uss_rl2, uss_rr1, uss_rr2, uss_left, uss_right, electrics, timer = host.init('sp1', False, True, True)
vehicle.sensors.poll('electrics', 'timer', 'state')

while electrics.data['running']:
    # print(electrics.data['accXSmooth'])
    vehicle.sensors.poll('electrics')
# lidar_data_readonly = lidar.stream()
# pos = np.array([vehicle.state['pos'][0], vehicle.state['pos'][1] - 2.25, vehicle.state['pos'][2] + 0.6])
# direction = vehicle.state['dir']

# lidar_data = lidar_data_readonly.copy()[:np.where(lidar_data_readonly == 0)[0][0]]
# lidar_data = lidar_data.reshape((len(lidar_data) // 3, 3))

# transform = np.identity(4)
# transform[:3, 3] = -pos

# lidar_data = np.column_stack((lidar_data, np.ones(len(lidar_data))))
# lidar_data = np.dot(lidar_data, transform.T)[:, :3]

# lidar_data = lidar_data[np.dot(lidar_data, direction) >= 0]

# print(lidar_data.shape)
# print(len(lidar_data.tobytes()))
# print(lidar_data)
# print(lidar_data.astype(np.float32))
# while electrics.data['running']:
#     vehicle.sensors.poll('electrics')
#     time.sleep(1)

# time.sleep(1)

# t = time.time()
# camera_data = camera.stream_colour(3686400)
# camera_data = np.array(camera_data).reshape(height, width, 4)
# camera_data = (0.299 * camera_data[:, :, 0] + 0.587 * camera_data[:, :, 1] + 0.114 * camera_data[:, :, 2]).astype(np.uint8)
# # camera_data = Image.fromarray(camera_data, 'L')
# img = camera_data.tobytes()
# print(time.time() - t)
# # transfer
# t2 = time.time()
# img = np.frombuffer(img, dtype=np.uint8, count=921600).reshape((720, 1280))
# print(time.time() - t2)

# t = time.time()
# camera_data = camera.stream_raw()['colour']
# print(time.time() - t)
# #transfer
# t2 = time.time()
# img = np.frombuffer(camera_data, dtype=np.uint8).reshape((921600, 4))
# img_rgb = img[:, :3]
# img_grey = np.dot(img_rgb, [0.2989, 0.5870, 0.1140]).astype(np.uint8).reshape((720, 1280))
# print(time.time() - t2)
# print(img_grey)
# print(img)

# t2 = time.time()
# img2 = np.frombuffer(img2, dtype=np.uint8).reshape((720, 1280))
# print(img2)
# print(time.time() - t2)

# print(len(img))
# print('separator')
# print(len(img2))

# for curr_dir in dirs:
#     home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer = host.init(curr_dir, len(curr_dir) == 3)

#     # input('Hit enter to start camera')
#     # camera_data = camera.stream_colour(3686400)
#     # camera_data = np.array(camera_data).reshape(height, width, 4)
#     # camera_data = (0.299 * camera_data[:, :, 0] + 0.587 * camera_data[:, :, 1] + 0.114 * camera_data[:, :, 2]).astype(np.uint8)
#     # Image.fromarray(camera_data, 'L').save('calibration.png', "PNG")
#     # break

#     time.sleep(60)
#     for i in range(0, 5):
#         time.sleep(60)
#         for j in range(0, 15):
#             bng.pause()
#             vehicle.sensors.poll('state')
#             lidar_data_readonly = lidar.stream()
#             pos = np.array([vehicle.state['pos'][0], vehicle.state['pos'][1] - 2.25, vehicle.state['pos'][2] + 0.6])
#             direction = vehicle.state['dir']

#             lidar_data = lidar_data_readonly.copy()[:np.where(lidar_data_readonly == 0)[0][0]]
#             lidar_data = lidar_data.reshape((len(lidar_data) // 3, 3))

#             transform = np.identity(4)
#             transform[:3, 3] = -pos

#             lidar_data = np.column_stack((lidar_data, np.ones(len(lidar_data))))
#             lidar_data = np.dot(lidar_data, transform.T)[:, :3]

#             vectors = lidar_data - np.array([0, 0, 0])
#             lidar_data = lidar_data[np.dot(vectors, direction) >= 0]

#             np.savetxt(f'{curr_dir}/sample{i}/lidar/pc{j}.txt', lidar_data, delimiter=' ')

#             if j % 3 == 0:
#                 camera_data = camera.stream_colour(3686400)
#                 camera_data = np.array(camera_data).reshape(height, width, 4)
#                 camera_data = (0.299 * camera_data[:, :, 0] + 0.587 * camera_data[:, :, 1] + 0.114 * camera_data[:, :, 2]).astype(np.uint8)
#                 Image.fromarray(camera_data, 'L').save(f'{curr_dir}/sample{i}/cam/img{j}.png', "PNG")

#             bng.resume()

# vehicle.sensors.poll('electrics')
# while(electrics.data['running']):
#     time.sleep(10)
#     vehicle.control(brake=0.6)
#     vehicle.sensors.poll('electrics')

# try:
#     OpenDriveExporter.compute_roads_and_junctions()
# except Exception as e:
#     pass
# OpenDriveExporter.export('road_network', bng)

# time.sleep(5)
# print('braking')
# # --- Emergency brake ---
# vehicle.sensors.poll('electrics')
# brake = electrics.data['brake_input']

# while abs(electrics.data['wheelspeed']) >= 0.05:
#     vehicle.control(throttle=0)
#     vehicle.control(brake=1)
#     vehicle.sensors.poll('electrics')

# vehicle.control(brake=0.0)
# vehicle.control(parkingbrake=0.0)
# # -----------------------

# print('done')

#  ----------- Timing benchmarks ---------------
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
# -----------------------------------------------

# plt.imsave('img.png', np.asarray(camera.poll()['colour'].convert('L')), cmap='gray')

# input('Hit enter to exit')

# vehicle.sensors.poll('state')
# data = lidar.poll()['pointCloud']
# vehLoc = vehicle.state['pos']
# i = 0
# # skip = False

# for a in data:
#     if a == 0:
#         break
#     i += 1
#     if i % 3 == 0:
#         # if skip == False:
#         #     lidar_file.write(str(a - vehLoc[(i - 1) % 3]) + '\n\n')
#         #     skip = True
#         # else:
#         #     skip = False
#         lidar_file.write(str(Decimal(a - vehLoc[(i - 1) % 3]).quantize(Decimal('0.001'), ROUND_HALF_UP)) + '\n')
#     else:
#         lidar_file.write(str(Decimal(a - vehLoc[(i - 1) % 3]).quantize(Decimal('0.001'), ROUND_HALF_UP)) + ', ')

# plt.imsave('img.png', np.asarray(data['colour'].convert('RGB')))

# LiDAR coordinates transformation testing
# Method 1 - 113 ms
# lidar_data = lidar_data_readonly.copy()[:np.where(lidar_data_readonly == 0)[0][0]]
# pos = vehicle.state['pos']

# for i in range(0, len(lidar_data)):
#     lidar_data[i] = lidar_data[i] - pos[i % 3]

# Method 2 - 35 ms
# lidar_data = lidar_data_readonly.copy()[:np.where(lidar_data_readonly == 0)[0][0]]
# lidar_data = lidar_data.reshape((len(lidar_data) // 3, 3))
# pos = np.array(vehicle.state['pos'])

# transform = np.identity(4)
# transform[:3, 3] = -pos

# lidar_data = np.column_stack((lidar_data, np.ones(len(lidar_data))))
# lidar_data = np.dot(lidar_data, transform.T)[:, :3]

# Sending camera over serial
# # Method 1 - 40.602 seconds
# camera_data.save('sh_mem.png', 'PNG')
# img = open('sh_mem.png', 'rb').read()
# ser.write(len(img).to_bytes(4) + img)
# # Method 2 - 79.903 seconds
# img = camera_data.tobytes()
# ser.write(len(img).to_bytes(4) + img)
# # Method 3 - 40.671 seconds
# stream = io.BytesIO()
# stream.seek(0)
# stream.truncate(0)
# camera_data.save(stream, 'PNG')
# img = stream.getvalue()
# ser.write(len(img).to_bytes(4) + img)
# stream.close()

# SSH testing - 500ms overhead
# ssh = paramiko.SSHClient()
# ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
# ssh.connect(pi_ip, username=user, password=passwd)
# stdin, stdout, stderr = ssh.exec_command("python3.11 BeamADAS/BeamADAS_Processor/comm.py")
# t1 = time.time()
# stdin.write(lidar_data.tobytes())
# stdin.channel.shutdown_write()
# print(stderr.read())
# print(time.time() - t1)
# ssh.close()

host.destroy(home, bng, scenario, vehicle, camera, lidar, uss_fl1, uss_fl2, uss_fr1, uss_fr2, uss_rl1, uss_rl2, uss_rr1, uss_rr2, uss_left, uss_right, electrics, timer)