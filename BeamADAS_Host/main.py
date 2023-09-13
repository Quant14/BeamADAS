# This script will gather sensor information from BeamNG.tech and send it to the external controller for processing
import sys
import os
import time
import matplotlib.pyplot as plt
import numpy as np
# np.set_printoptions(threshold=sys.maxsize)

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Lidar, Camera

# Open BeamNG.tech home directory
home = open(os.path.join(os.getcwd(), "bngtechdir.txt"), "r")
lidar_file = open(os.path.join(os.getcwd(), "lidar.txt"), "w")

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:4771
bng = BeamNGpy('localhost', 4771, home=home.readline())

bng.open()

# scenario = Scenario('italy', 'test')
scenario = Scenario('garage_v2', 'test')

vehicle = Vehicle('ego_vehicle', model='etk800', license='TEST', color=(0.1, 0.5, 0.1, 1))

# scenario.add_vehicle(vehicle, pos=(1199, -830, 146), rot_quat=(-0.2810479, -0.0583663, -0.9556557, 0.0657856))
scenario.add_vehicle(vehicle, pos=(-0.342, -0.044, 100.206), rot_quat=(0, 0, 0, 1))
scenario.make(bng)

bng.scenario.load(scenario)
bng.settings.set_deterministic(40)
bng.scenario.start()

lidar = Lidar('lidar', 
              bng, 
              vehicle, 
              requested_update_time=0.03, 
              pos=(0, -2.3, 0.6), 
              dir=(-1, 0.15, 0), 
              vertical_resolution=2, 
              vertical_angle=20, 
              frequency=30, 
              horizontal_angle=30,
              max_distance=150,
              is_visualised=True)
# camera = Camera('camera', 
#                 bng, 
#                 vehicle, 
#                 requested_update_time=0.03, 
#                 pos=(0, -0.35, 1.3), 
#                 resolution=(1280, 720), 
#                 field_of_view_y=60, 
#                 near_far_planes=(0.05, 200), 
#                 is_render_colours=True, 
#                 is_render_annotations=False, 
#                 is_render_depth=False)

time.sleep(5)

# input('Hit enter to start camera')
# for i in range(0, 30, 1):
#     data = camera.poll()
#     plt.imsave('img' + str(i) + '.png', np.asarray(data['colour'].convert('RGB')))

# input('Hit enter to take final frame')

data = lidar.poll()['pointCloud']
i = 0
skip = False
for a in data:
    if a == 0:
        break
    i += 1
    if i % 6 == 0:
        lidar_file.write(str(a) + '\n\n')
        skip = True
    elif i % 3 == 0:
        if skip == False:
            lidar_file.write(str(a) + '\n\n')
        skip = False
    elif skip == False:
        lidar_file.write(str(a) + '\n')

# data = camera.poll()
# plt.imsave('img30.png', np.asarray(data['colour'].convert('RGB')))

lidar_file.close()
lidar.remove()
# camera.remove()
bng.close()
home.close()