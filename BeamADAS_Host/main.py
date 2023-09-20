# This script will gather sensor information from BeamNG.tech and send it to the external controller for processing
import sys
import os
import time
import matplotlib.pyplot as plt
import numpy as np
from decimal import Decimal, ROUND_HALF_UP

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Lidar, Camera, Ultrasonic

# Open BeamNG.tech home directory
home = open(os.path.join(os.getcwd(), "bngtechdir.txt"), "r")
lidar_file = open(os.path.join(os.getcwd(), "lidar.txt"), "w")

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:4771
bng = BeamNGpy('localhost', 4771, home=home.readline())

bng.open()

scenario = Scenario('italy', 'test')
# scenario = Scenario('smallgrid', 'test')

vehicle = Vehicle('ego_vehicle', model='etk800', license='ADAS', color=(0.1, 0.5, 0.1, 1))
# box = Vehicle('box', model='metal_box')

scenario.add_vehicle(vehicle, pos=(1199, -830, 146), rot_quat=(-0.2810479, -0.0583663, -0.9556557, 0.0657856))
# scenario.add_vehicle(vehicle, pos=(0, 0, 0.206))
# scenario.add_vehicle(box, pos=(0, -5, 0))
scenario.make(bng)

bng.scenario.load(scenario)
bng.settings.set_deterministic(30)
bng.scenario.start()

camera = Camera('camera', 
                bng, 
                vehicle, 
                requested_update_time=0.03, 
                pos=(0, -0.35, 1.3), 
                resolution=(1280, 720), 
                field_of_view_y=60, 
                near_far_planes=(0.05, 200), 
                is_render_colours=True, 
                is_render_annotations=False, 
                is_render_depth=False)
uss_f = Ultrasonic('uss_f', 
                bng, 
                vehicle, 
                requested_update_time=0.3, 
                pos=(0, -2.3, 0.4), 
                dir=(0, -1, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 3.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=3.0,
                sensitivity=0.005, 
                is_visualised=True)
uss_fl = Ultrasonic('uss_fl', 
                bng, 
                vehicle, 
                requested_update_time=0.3, 
                pos=(0.78, -2.0, 0.4), 
                dir=(0.5, -0.5, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 3.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=3.0,
                sensitivity=0.005, 
                is_visualised=True)
uss_fr = Ultrasonic('uss_fr', 
                bng, 
                vehicle, 
                requested_update_time=0.3, 
                pos=(-0.78, -2.0, 0.4), 
                dir=(-0.5, -0.5, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 3.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=3.0,
                sensitivity=0.005, 
                is_visualised=True)
uss_r = Ultrasonic('uss_r', 
                bng, 
                vehicle, 
                requested_update_time=0.3, 
                pos=(0, 2.4, 0.4), 
                dir=(0, 1, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 3.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=3.0,
                sensitivity=0.005, 
                is_visualised=True)
uss_rl = Ultrasonic('uss_rl', 
                bng, 
                vehicle, 
                requested_update_time=0.3, 
                pos=(0.8, 2.1, 0.4), 
                dir=(0.5, 0.5, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 3.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=3.0,
                sensitivity=0.005, 
                is_visualised=True)
uss_rr = Ultrasonic('uss_rr', 
                bng, 
                vehicle, 
                requested_update_time=0.3, 
                pos=(-0.8, 2.1, 0.4), 
                dir=(-0.5, 0.5, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 3.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=3.0,
                sensitivity=0.005, 
                is_visualised=True)
lidar = Lidar('lidar', 
              bng, 
              vehicle, 
              requested_update_time=0.03, 
              pos=(0, -2.25, 0.60), 
              dir=(-1, 0.1, 0), 
              vertical_resolution=60, 
              vertical_angle=20, 
              rays_per_second=172800,
              frequency=30, 
              horizontal_angle=30,
              max_distance=150,
              is_visualised=False)
# lidar = Lidar('lidar', 
#               bng, 
#               vehicle, 
#               requested_update_time=0.03, 
#               pos=(0, -2.3, 0.6), 
#               dir=(-1, 0.15, 0), 
#               vertical_resolution=10, 
#               vertical_angle=20, 
#               rays_per_second=4500,
#               frequency=30, 
#               horizontal_angle=30,
#               max_distance=150,
#               is_visualised=True)
time.sleep(5)

# input('Hit enter to start camera')
# for i in range(0, 30, 1):
#     data = camera.poll()
#     plt.imsave('img' + str(i) + '.png', np.asarray(data['colour'].convert('RGB')))

scenario.update()
data = lidar.poll()['pointCloud']
vehLoc = vehicle.state['pos']
i = 0
# skip = False

input('Hit enter to exit')

for a in data:
    if a == 0:
        break
    i += 1
    if i % 3 == 0:
        # if skip == False:
        #     lidar_file.write(str(a - vehLoc[(i - 1) % 3]) + '\n\n')
        #     skip = True
        # else:
        #     skip = False
        lidar_file.write(str(Decimal(a - vehLoc[(i - 1) % 3]).quantize(Decimal('0.001'), ROUND_HALF_UP)) + '\n')
    else:
        lidar_file.write(str(Decimal(a - vehLoc[(i - 1) % 3]).quantize(Decimal('0.001'), ROUND_HALF_UP)) + ', ')

data = camera.poll()['colour']
plt.imsave('img.png', np.asarray(data.convert('L')), cmap='gray')
# plt.imsave('img.png', np.asarray(data['colour'].convert('RGB')))

lidar_file.close()
lidar.remove()
camera.remove()
uss_f.remove()
uss_fl.remove()
uss_fr.remove()
uss_r.remove()
uss_rl.remove()
uss_rr.remove()
bng.close()
home.close()