# This file includes the host init and destroy code for a cleaner main file

import os
# import pyserial

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, Lidar, Ultrasonic, Electrics, Timer

def init(sp, traffic):
    # Open BeamNG.tech home directory
    home = open(os.path.join(os.getcwd(), "bngtechdir.txt"), "r")

    # Instantiate BeamNGpy instance running the simulator from the given path,
    # communicating over localhost:4771
    bng = BeamNGpy('localhost', 4771, home=home.readline())

    bng.open()

    # Create vehicles
    vehicle = Vehicle('ego_vehicle', model='etk800', license='ADAS', color=(0.31, 0.33, 0.24, 1))

    # Load map and spawn vehicle
    if sp == 'sp1' or sp == 'sp1_no_traffic':
        scenario = Scenario('italy', 'test')
        scenario.add_vehicle(vehicle, pos=(1216.629, -824.389, 145.414), rot_quat=(-0.014, 0.012, -0.518, 0.855)) # SP1
    elif sp == 'sp2' or sp == 'sp2_no_traffic':
        scenario = Scenario('italy', 'test')
        scenario.add_vehicle(vehicle, pos=(-1331.383, 1565.515, 152.679), rot_quat=(0, 0.005, 0.639, 0.769)) # SP2
    elif sp == 'sp0':
        scenario = Scenario('smallgrid', 'test')
        scenario.add_vehicle(vehicle, pos=(0, 0, 0.206))
    
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
            requested_update_time=0.09,
            pos=(0, -0.35, 1.3), 
            resolution=(1280, 720), 
            field_of_view_y=55,
            near_far_planes=(0.1, 100), 
            is_render_colours=True, 
            is_render_annotations=False, 
            is_render_depth=False,
            is_streaming=True,
            is_using_shared_memory=True)
    lidar = Lidar('lidar', 
                bng, 
                vehicle, 
                requested_update_time=0.03,
                pos=(0, -2.25, 0.60), 
                dir=(-1, 0.1, 0), 
                vertical_resolution=20, 
                vertical_angle=5, 
                rays_per_second=18000,
                frequency=30, 
                horizontal_angle=20,
                max_distance=100,
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

    # Set AI driver
    vehicle.ai_set_speed(22.222, 'limit')
    vehicle.ai_drive_in_lane(True)
    vehicle.ai_set_mode('span')

    # Set AI traffic
    if traffic:
        bng.spawn_traffic()

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