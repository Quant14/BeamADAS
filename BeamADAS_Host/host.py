# This file includes the host init and destroy code for a cleaner main file

import os
# import pyserial

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, Lidar, Ultrasonic, Electrics, Timer

def init():
    # Open BeamNG.tech home directory
    home = open(os.path.join(os.getcwd(), "bngtechdir.txt"), "r")

    # Instantiate BeamNGpy instance running the simulator from the given path,
    # communicating over localhost:4771
    bng = BeamNGpy('localhost', 4771, home=home.readline())

    bng.open()

    # Set map
    scenario = Scenario('italy', 'test')
    # scenario = Scenario('smallgrid', 'test')

    # Create vehicles
    vehicle = Vehicle('ego_vehicle', model='etk800', license='ADAS', color=(0.31, 0.33, 0.24, 1))
    # box = Vehicle('box', model='metal_box')

    # Add vehicles to scenario
    scenario.add_vehicle(vehicle, pos=(1216.629, -824.389, 145.414), rot_quat=(-0.014, 0.012, -0.518, 0.855)) # SP1
    # scenario.add_vehicle(vehicle, pos=(0, 0, 0.206))
    # scenario.add_vehicle(box, pos=(0, -5, 0))
    
    # Load scenario
    scenario.make(bng)
    bng.scenario.load(scenario)

    # Start BeamNG
    bng.settings.set_deterministic(30)
    bng.scenario.start()

    # Init sensors
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
            is_render_depth=False,
            is_streaming=True,
            is_using_shared_memory=True)
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
                is_visualised=False,
                is_streaming=True)
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
                    is_visualised=False,
                    is_streaming=True)
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
                    is_visualised=False,
                    is_streaming=True)
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
                    is_visualised=False,
                    is_streaming=True)
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
                    is_visualised=False,
                    is_streaming=True)
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
                    is_visualised=False,
                    is_streaming=True)
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
                    is_visualised=False,
                    is_streaming=True)
    uss_left = Ultrasonic('uss_left',
                    bng,
                    vehicle,
                    requested_update_time=0.06,
                    pos=(0.95, -0.4, 0.95),
                    dir=(0.6, 0.4, 0),
                    field_of_view_y=60,
                    near_far_planes=(0.1, 3.0), 
                    range_min_cutoff=0.1, 
                    range_direct_max_cutoff=3.0,
                    sensitivity=0.01, 
                    is_visualised=False,
                    is_streaming=True)
    uss_right = Ultrasonic('uss_right',
                    bng,
                    vehicle,
                    requested_update_time=0.06,
                    pos=(-0.95, -0.4, 0.95),
                    dir=(-0.6, 0.4, 0),
                    field_of_view_y=60,
                    near_far_planes=(0.1, 3.0), 
                    range_min_cutoff=0.1, 
                    range_direct_max_cutoff=3.0,
                    sensitivity=0.01, 
                    is_visualised=False,
                    is_streaming=True)

    electrics = Electrics()
    vehicle.attach_sensor('electrics', electrics)
    electrics.attach(vehicle, 'electrics')
    electrics.connect(bng, vehicle)

    timer = Timer()
    vehicle.attach_sensor('timer', timer)
    timer.attach(vehicle, 'timer')
    timer.connect(bng, vehicle)

    return home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer

def is_elapsed(timer, vehicle, timestamp):
    vehicle.sensors.poll('timer')
    return timer.data['time'] - timestamp >= 0.06

def destroy(home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer):
    camera.remove()
    lidar.remove()
    uss_f.remove()
    uss_fl.remove()
    uss_fr.remove()
    uss_r.remove()
    uss_rl.remove()
    uss_rr.remove()
    uss_left.remove()
    uss_right.remove()
    vehicle.detach_sensor('electrics')
    electrics.detach(vehicle, 'electrics')
    electrics.disconnect(bng, vehicle)
    vehicle.detach_sensor('timer')
    timer.detach(vehicle, 'timer')
    timer.disconnect(bng, vehicle)
    bng.close()
    home.close()