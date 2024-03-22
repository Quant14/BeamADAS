# This script will gather sensor information from BeamNG.tech and send it to the external controller for processing

import host

import time
import numpy as np

width = 1280
height = 720

import socket as sock
from multiprocessing import Process, Event
import traceback
import random

def sender(ready_event, ready_cam_event, ready_lidar_event, ready_uss_event, ready_blind_event, exit_event):
    home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer = host.init('sp1', False, False, True)

    adas_state = 0 # states: 0 = off, 1 = on, 2 = ready, 3 = active
    prev_state = 0
    # Initialize simulation
    print('Initializing simulation...')
    adas_state = 1

    vehicle.sensors.poll('electrics')
    second = 0

    print('Starting receivers...')
    ready_event.set()

    # Check connection with Pi4 to set adas_state to ready
    print('Connecting to Pi4...')

    socket = sock.socket(sock.AF_INET, sock.SOCK_STREAM)
    socket.connect(("169.254.151.166", 4441))

    try:
        print('Waiting for cam receiver...')
        ready_cam_event.wait()
        print('Waiting for lidar receiver...')
        ready_lidar_event.wait()
        print('Waiting for uss receiver...')
        ready_uss_event.wait()
        print('Waiting for blind spot receiver...')
        ready_blind_event.wait()

        print('Initiating Pi4...')
        host.send_data(socket, b'I', None, None, None, b'init')

        time.sleep(5)

        print('Simulation running.')
        while(electrics.data['running']):
            adas_state = 2
            bng.pause()

            # Update misc data
            vehicle.sensors.poll('electrics')
            speed = electrics.data['wheelspeed']

            host.send_data(socket, b'S', None, None, None, speed)

            # Get ADAS sensors data
            if speed > 5.555 and not electrics.data['hazard_signal'] and not electrics.data['left_signal'] and not electrics.data['right_signal']: # Speed for LiDAR
                vehicle.sensors.poll('state', 'timer')
                lidar_data_readonly = lidar.stream()
                pos = np.array([vehicle.state['pos'][0], vehicle.state['pos'][1] - 2.25, vehicle.state['pos'][2] + 0.6])
                direction = vehicle.state['dir']

                lidar_data = lidar_data_readonly.copy()[:np.where(lidar_data_readonly == 0)[0][0]]

                new_shape = (len(lidar_data) // 3, 3)
                lidar_data = lidar_data[:np.prod(new_shape)]
                lidar_data = lidar_data.reshape(new_shape)

                transform = np.identity(4)
                transform[:3, 3] = -pos

                lidar_data = np.column_stack((lidar_data, np.ones(len(lidar_data))))
                lidar_data = np.dot(lidar_data, transform.T)[:, :3]

                lidar_data = lidar_data[np.dot(lidar_data, direction) >= 0].astype(np.float32)
                direction = np.array(direction[:2], dtype=np.float32)
                host.send_data(socket, b'L', timer['time'], direction[:2], None, lidar_data.tobytes())
                adas_state = 3

            if second % 2 == 0:
                if speed > 11.111 and not electrics.data['hazard_signal'] and not electrics.data['left_signal'] and not electrics.data['right_signal']: # Speed for camera
                    vehicle.sensors.poll('timer')
                    camera_data = camera.stream_colour(3686400)
                    camera_data = np.array(camera_data).reshape(height, width, 4)
                    camera_data = (0.299 * camera_data[:, :, 0] + 0.587 * camera_data[:, :, 1] + 0.114 * camera_data[:, :, 2]).astype(np.uint8)

                    img = camera_data.tobytes()

                    host.send_data(socket, b'C', timer['time'], None, None, img)
                    adas_state = 3
                elif speed < 5.555: # Parking speed
                    vehicle.sensors.poll('electrics', 'timer')
                    park_data = np.array([uss_f.stream()[0], uss_fl.stream()[0], uss_fr.stream()[0],
                            uss_r.stream()[0], uss_rl.stream()[0], uss_rr.stream()[0]], dtype=np.float32)
                    host.send_data(socket, b'P', timer['time'], None, electrics.data['gear'].encode(), park_data.tobytes())
                    adas_state = 3

                if second % 6 == 0:
                    # Blind spot detection
                    blind_data = np.array([uss_left.stream(), uss_right.stream()], dtype=np.float32)
                    host.send_data(socket, b'B', None, None, None, blind_data.tobytes())

                if second % 30 == 0:
                    if prev_state == 3 and adas_state == 2:
                        vehicle.control(throttle=100)
                        vehicle.control(brake=0)

                    prev_state = adas_state

            bng.resume()
            second += 1

    except Exception as e:
        traceback.print_exc()
    finally:
        host.send_data(socket, b'Q', None, None, None, b'')
        exit_event.set()
        print('Exiting...')
        adas_state = 0
        time.sleep(3)
        socket.close()
        host.destroy(home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer)

def receiver(proc, ready_event, ready_back_event, exit_event):
    try:
        print(f'Receiver {proc} waiting for sender...')
        ready_event.wait()
        print(f'Starting receiver {proc}...')
        time.sleep(random.random() % 2)
        home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer = host.init('sp0', False, False, False)

        print(f'Receiver {proc} opening connection...')

        socket = sock.socket(sock.AF_INET, sock.SOCK_STREAM)
        socket.bind(('0.0.0.0', 4441 + proc))
        ready_back_event.set()
        print('Waiting for connection...')
        socket.listen()
        socket.settimeout(20)
        conn, addr = socket.accept()
        print(f'Receiver {proc} operational...')
        while not exit_event.is_set():
            t = time.time()
            response_type, response_1, response_2 = host.recv_data(conn)
            if time.time() - t >= 20:
                if exit_event.is_set():
                    break
                else:
                    continue
            if response_type == b'I':
                # print('Braking!')
                vehicle.control(throttle=response_1)
                vehicle.control(brake=response_2)
            elif response_type == b'B':
                # print('Checking blind spots')
                if response_1:
                    print('blind spot left')
                if response_2:
                    print('blind spot right')
    except Exception as e:
        traceback.print_exc()
    finally:
        print(f'Receiver {proc} exiting...')
        socket.close()

def main():
    ready_event = Event()
    ready_cam_event = Event()
    ready_lidar_event = Event()
    ready_uss_event = Event()
    ready_blind_event = Event()
    exit_event = Event()

    sender_proc = Process(target=sender, args=(ready_event, ready_cam_event, ready_lidar_event, ready_uss_event, ready_blind_event, exit_event))
    cam_proc = Process(target=receiver, args=(1, ready_event, ready_cam_event, exit_event))
    lidar_proc = Process(target=receiver, args=(2, ready_event, ready_lidar_event, exit_event))
    uss_proc = Process(target=receiver, args=(3, ready_event, ready_uss_event, exit_event))
    blind_proc = Process(target=receiver, args=(4, ready_event, ready_blind_event, exit_event))

    sender_proc.start()
    cam_proc.start()
    lidar_proc.start()
    uss_proc.start()
    blind_proc.start()

    sender_proc.join()
    cam_proc.join()
    lidar_proc.join()
    uss_proc.join()
    blind_proc.join()

if __name__ == '__main__':
    main()