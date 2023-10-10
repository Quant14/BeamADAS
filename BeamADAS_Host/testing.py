# This script will gather sensor information from BeamNG.tech and send it to the external controller for processing

import sys
import os
import time
import matplotlib.pyplot as plt
import numpy as np
from decimal import Decimal, ROUND_HALF_UP

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, Lidar, Ultrasonic, Electrics, Timer

# Open BeamNG.tech home directory
home = open(os.path.join(os.getcwd(), "bngtechdir.txt"), "r")
lidar_file = open(os.path.join(os.getcwd(), "lidar.txt"), "w")

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:4771
bng = BeamNGpy('localhost', 4771, home=home.readline())

bng.open()

# scenario = Scenario('italy', 'test')
scenario = Scenario('smallgrid', 'test')

vehicle = Vehicle('ego_vehicle', model='etk800', license='ADAS', color=(0.1, 0.5, 0.1, 1))
# box = Vehicle('box', model='metal_box')

# scenario.add_vehicle(vehicle, pos=(1205, -824, 146), rot_quat=(-0.278, -0.025, -0.953, 0.302))
scenario.add_vehicle(vehicle, pos=(0, 0, 0.206))
# scenario.add_vehicle(box, pos=(0, -5, 0))
scenario.make(bng)

bng.scenario.load(scenario)
bng.settings.set_deterministic(30)
bng.scenario.start()

camera = Camera('camera', 
                bng, 
                vehicle, 
                requested_update_time=0.06,
                update_priority=1,
                pos=(0, -0.35, 1.3), 
                resolution=(1280, 720), 
                field_of_view_y=60, 
                near_far_planes=(0.05, 200), 
                is_render_colours=True, 
                is_render_annotations=False, 
                is_render_depth=False)
lidar = Lidar('lidar', 
              bng, 
              vehicle, 
              requested_update_time=0.03,
              update_priority=1,
              pos=(0, -2.25, 0.60), 
              dir=(-1, 0.1, 0), 
              vertical_resolution=60, 
              vertical_angle=20, 
              rays_per_second=172800,
              frequency=30, 
              horizontal_angle=30,
              max_distance=150,
              is_visualised=True)
uss_f = Ultrasonic('uss_f', 
                bng, 
                vehicle, 
                requested_update_time=0.06,
                pos=(0, -2.3, 0.4), 
                dir=(0, -1, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 8.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=8.0,
                sensitivity=0.005, 
                is_visualised=False)
uss_fl = Ultrasonic('uss_fl', 
                bng, 
                vehicle, 
                requested_update_time=0.06,
                pos=(0.78, -2.0, 0.4), 
                dir=(0.5, -0.5, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 3.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=3.0,
                sensitivity=0.005, 
                is_visualised=False)
uss_fr = Ultrasonic('uss_fr', 
                bng, 
                vehicle, 
                requested_update_time=0.06,
                pos=(-0.78, -2.0, 0.4), 
                dir=(-0.5, -0.5, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 3.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=3.0,
                sensitivity=0.005, 
                is_visualised=False)
uss_r = Ultrasonic('uss_r', 
                bng, 
                vehicle, 
                requested_update_time=0.06,
                pos=(0, 2.4, 0.4), 
                dir=(0, 1, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 3.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=3.0,
                sensitivity=0.005, 
                is_visualised=False)
uss_rl = Ultrasonic('uss_rl', 
                bng, 
                vehicle, 
                requested_update_time=0.06,
                pos=(0.8, 2.1, 0.4), 
                dir=(0.5, 0.5, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 3.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=3.0,
                sensitivity=0.005, 
                is_visualised=False)
uss_rr = Ultrasonic('uss_rr', 
                bng, 
                vehicle, 
                requested_update_time=0.06,
                pos=(-0.8, 2.1, 0.4), 
                dir=(-0.5, 0.5, 0), 
                field_of_view_y=45, 
                near_far_planes=(0.1, 3.0), 
                range_min_cutoff=0.1, 
                range_direct_max_cutoff=3.0,
                sensitivity=0.005, 
                is_visualised=False)

electrics = Electrics()
vehicle.attach_sensor('electrics', electrics)
electrics.attach(vehicle, 'electrics')
electrics.connect(bng, vehicle)

timer = Timer()
vehicle.attach_sensor('timer', timer)
timer.attach(vehicle, 'timer')
timer.connect(bng, vehicle)

# input('Hit enter to start camera')
# time.sleep(5)
# for i in range(0, 30, 1):
#     plt.imsave('img' + str(i) + '.png', np.asarray(camera.poll()['colour'].convert('L')), cmap='gray')

# vehicle.ai_set_speed(22, 'limit')
# vehicle.ai_drive_in_lane(True)
# vehicle.ai_set_mode('span')

# time.sleep(15)

# # --- Emergency brake ---
# vehicle.sensors.poll('electrics')
# brake = electrics.data['brake_input']

# while abs(electrics.data['wheelspeed']) >= 0.05:
#     vehicle.control(throttle=0)
#     vehicle.control(brake=1)
#     vehicle.sensors.poll('electrics')

# vehicle.control(brake=brake)
# vehicle.control(parkingbrake=1.0)
# # -----------------------

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

input('Hit enter to exit')

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

camera.remove()
lidar_file.close()
lidar.remove()
uss_f.remove()
uss_fl.remove()
uss_fr.remove()
uss_r.remove()
uss_rl.remove()
uss_rr.remove()
vehicle.detach_sensor('electrics')
electrics.detach(vehicle, 'electrics')
electrics.disconnect(bng, vehicle)
vehicle.detach_sensor('timer')
timer.detach(vehicle, 'timer')
timer.disconnect(bng, vehicle)
bng.close()
home.close()