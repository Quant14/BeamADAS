# This file includes the host init and destroy code for a cleaner main file

import os
import socket as sock
import struct

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, Lidar, Ultrasonic, Electrics, Timer

def init(sp, ai, traffic, launch):
    # Open BeamNG.tech home directory
    home = open(os.path.join(os.getcwd(), "bngtechdir.txt"), "r")

    # Instantiate BeamNGpy instance running the simulator from the given path,
    # communicating over localhost:4441
    bng = BeamNGpy('localhost', 4441, home=home.readline())

    bng.open(launch=launch)
    if launch:
        # Create vehicles
        vehicle = Vehicle('ego_vehicle', model='etk800', license='ADAS', color=(0.31, 0.33, 0.24, 1))

        # Load map and spawn vehicle
        if sp == 'sp1' or sp == 'sp1_no_traffic':
            scenario = Scenario('italy', 'BeamADAS')
            scenario.add_vehicle(vehicle, pos=(1216.629, -824.389, 145.414), rot_quat=(-0.014, 0.012, -0.518, 0.855)) # SP1
        elif sp == 'sp2' or sp == 'sp2_no_traffic':
            scenario = Scenario('italy', 'BeamADAS')
            scenario.add_vehicle(vehicle, pos=(-1331.383, 1565.515, 152.679), rot_quat=(0, 0.005, 0.639, 0.769)) # SP2
        elif sp == 'sp0':
            scenario = Scenario('smallgrid', 'BeamADAS')
            scenario.add_vehicle(vehicle, pos=(0, 0, 0.206))
            vehicle2 = Vehicle('lidar_vehicle', model='etki', license='LIDAR', color=(0.5, 0.5, 0.5, 1))
            scenario.add_vehicle(vehicle2, pos=(0, -150, 0.206))

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
                    requested_update_time=0.0666,
                    pos=(0, -0.35, 1.3),
                    resolution=(1280, 720),
                    field_of_view_y=55,
                    near_far_planes=(0.1, 100),
                    is_render_colours=True,
                    is_render_depth=False,
                    is_render_annotations=False,
                    is_render_instance=False,
                    is_streaming=True,
                    is_using_shared_memory=True)
        lidar = Lidar('lidar',
                    bng,
                    vehicle,
                    requested_update_time=0.0333,
                    pos=(0, -2.25, 0.60),
                    dir=(-1, 0.075, 0),
                    vertical_resolution=20,
                    vertical_angle=5,
                    rays_per_second=18000,
                    frequency=30,
                    horizontal_angle=20,
                    max_distance=100,
                    is_visualised=False,
                    is_streaming=True)
        uss_fl1 = Ultrasonic('uss_fl1',
                        bng,
                        vehicle,
                        requested_update_time=0.067,
                        pos=(0.25, -2.28, 0.4),
                        dir=(0.15, -0.85, 0.1),
                        field_of_view_y=30,
                        near_far_planes=(0.1, 3.0),
                        range_min_cutoff=0.1,
                        range_direct_max_cutoff=3.0,
                        sensitivity=0.003,
                        is_visualised=False,
                        is_streaming=True)
        uss_fl2 = Ultrasonic('uss_fl2',
                        bng,
                        vehicle,
                        requested_update_time=0.067,
                        pos=(0.78, -2.0, 0.4),
                        dir=(0.5, -0.5, 0.1),
                        field_of_view_y=30,
                        near_far_planes=(0.1, 3.0),
                        range_min_cutoff=0.1,
                        range_direct_max_cutoff=3.0,
                        sensitivity=0.003,
                        is_visualised=False,
                        is_streaming=True)
        uss_fr1 = Ultrasonic('uss_fr1',
                        bng,
                        vehicle,
                        requested_update_time=0.067,
                        pos=(-0.25, -2.28, 0.4),
                        dir=(-0.15, -0.85, 0.1),
                        field_of_view_y=30,
                        near_far_planes=(0.1, 3.0),
                        range_min_cutoff=0.1,
                        range_direct_max_cutoff=3.0,
                        sensitivity=0.003,
                        is_visualised=False,
                        is_streaming=True)
        uss_fr2 = Ultrasonic('uss_fr2',
                        bng,
                        vehicle,
                        requested_update_time=0.067,
                        pos=(-0.78, -2.0, 0.4),
                        dir=(-0.5, -0.5, 0.1),
                        field_of_view_y=30,
                        near_far_planes=(0.1, 3.0),
                        range_min_cutoff=0.1,
                        range_direct_max_cutoff=3.0,
                        sensitivity=0.003,
                        is_visualised=False,
                        is_streaming=True)
        uss_rl1 = Ultrasonic('uss_rl1',
                        bng,
                        vehicle,
                        requested_update_time=0.067,
                        pos=(0.25, 2.4, 0.4),
                        dir=(0.15, 0.85, 0.1),
                        field_of_view_y=30,
                        near_far_planes=(0.1, 3.0),
                        range_min_cutoff=0.1,
                        range_direct_max_cutoff=3.0,
                        sensitivity=0.003,
                        is_visualised=False,
                        is_streaming=True)
        uss_rl2 = Ultrasonic('uss_rl2',
                        bng,
                        vehicle,
                        requested_update_time=0.067,
                        pos=(0.8, 2.1, 0.4),
                        dir=(0.5, 0.5, 0.1),
                        field_of_view_y=30,
                        near_far_planes=(0.1, 3.0),
                        range_min_cutoff=0.1,
                        range_direct_max_cutoff=3.0,
                        sensitivity=0.003,
                        is_visualised=False,
                        is_streaming=True)
        uss_rr1 = Ultrasonic('uss_rr1',
                        bng,
                        vehicle,
                        requested_update_time=0.067,
                        pos=(-0.25, 2.4, 0.4),
                        dir=(-0.15, 0.85, 0.1),
                        field_of_view_y=30,
                        near_far_planes=(0.1, 3.0),
                        range_min_cutoff=0.1,
                        range_direct_max_cutoff=3.0,
                        sensitivity=0.003,
                        is_visualised=False,
                        is_streaming=True)
        uss_rr2 = Ultrasonic('uss_rr2',
                        bng,
                        vehicle,
                        requested_update_time=0.067,
                        pos=(-0.8, 2.1, 0.4),
                        dir=(-0.5, 0.5, 0.1),
                        field_of_view_y=30,
                        near_far_planes=(0.1, 3.0),
                        range_min_cutoff=0.1,
                        range_direct_max_cutoff=3.0,
                        sensitivity=0.003,
                        is_visualised=False,
                        is_streaming=True)
        uss_left = Ultrasonic('uss_left',
                        bng,
                        vehicle,
                        requested_update_time=0.198,
                        pos=(0.95, -0.4, 0.95),
                        dir=(0.6, 0.4, 0),
                        field_of_view_y=45,
                        near_far_planes=(0.1, 3.0),
                        range_min_cutoff=0.1,
                        range_direct_max_cutoff=3.0,
                        sensitivity=0.01,
                        is_visualised=False,
                        is_streaming=True)
        uss_right = Ultrasonic('uss_right',
                        bng,
                        vehicle,
                        requested_update_time=0.198,
                        pos=(-0.95, -0.4, 0.95),
                        dir=(-0.6, 0.4, 0),
                        field_of_view_y=45,
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
        if ai:
            vehicle.ai_set_speed(22.222, 'limit')
            vehicle.ai_drive_in_lane(True)
            vehicle.ai_set_mode('span')

        # Set AI traffic
        if traffic:
            bng.spawn_traffic()

    else:
        scenario = bng.scenario.get_current()
        vehicle = bng.vehicles.get_current()['ego_vehicle']
        vehicle.connect(bng)

        return home, bng, scenario, vehicle, None, None, None, None, None, None, None, None, None, None, None, None

    return home, bng, scenario, vehicle, camera, lidar, uss_fl1, uss_fl2, uss_fr1, uss_fr2, uss_rl1, uss_rl2, uss_rr1, uss_rr2, uss_left, uss_right, electrics, timer

def destroy(home, bng, scenario, vehicle, camera, lidar, uss_fl1, uss_fl2, uss_fr1, uss_fr2, uss_rl1, uss_rl2, uss_rr1, uss_rr2, uss_left, uss_right, electrics, timer):
    camera.remove()
    lidar.remove()
    uss_fl1.remove()
    uss_fl2.remove()
    uss_fr1.remove()
    uss_fr2.remove()
    uss_rl1.remove()
    uss_rl2.remove()
    uss_rr1.remove()
    uss_rr2.remove()
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

def send_data(socket, type, timestamp, dir, gear, accX, data):
    if type == b'S':
        socket.sendall(struct.pack('>cf', type, data))
    elif type == b'C':
        socket.sendall(struct.pack('>cf', type, timestamp) + data)
    elif type == b'L':
        socket.sendall(struct.pack('>cIffff', type, len(data), timestamp, dir[0], dir[1], accX) + data)
    elif type == b'P':
        socket.sendall(struct.pack('>cfc', type, timestamp, gear) + data)
    else:
        socket.sendall(struct.pack('>cI', type, len(data)) + data)

def recv_data(socket):
    try:
        recv = socket.recv(1)
    except socket.timeout:
        return None, None, None
    if len(recv) < 1: return None, None, None

    data_type = struct.unpack('>c', recv)[0]
    response_1 = None
    response_2 = None

    if data_type == b'I':
        recv = socket.recv(8)
        if len(recv) < 8: return None, None, None
        response_1, response_2 = struct.unpack('>ff', recv)
    elif data_type == b'B':
        recv = socket.recv(2)
        if len(recv) < 2: return None, None, None
        response_1, response_2 = struct.unpack('>??', recv)
    return data_type, response_1, response_2