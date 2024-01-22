# This script will gather sensor information from BeamNG.tech and send it to the external controller for processing

import host

import time
import matplotlib.pyplot as plt
import numpy as np

from PIL import Image
width = 1280
height = 720

import socket
from multiprocessing import Process, Event, Array, Value

def sender(ready_event, ready_cam_event, ready_lidar_event, exit_event):
    home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer = host.init('sp0', False, True)

    adas_state = 0 # states: 0 = off, 1 = on, 2 = ready, 3 = active
    # Initialize simulation
    print('Initializing simulation...')
    adas_state = 1

    vehicle.sensors.poll('electrics')
    second = 0

    print('Starting receiver...')
    ready_event.set()

    # Check connection with Pi4 to set adas_state to ready
    print('Connecting to Pi4...')

    socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket.connect(("169.254.151.166", 4441))

    adas_state = 2

    try: 
        print('Waiting for cam receiver...')
        ready_cam_event.wait()
        print('Waiting for lidar receiver...')
        ready_lidar_event.wait()
        print('Initiating data processing...')
        host.send_data(socket, 'I', b'')
        print('Simulation running.')
        while(electrics.data['running']):
            bng.pause()

            # Update misc data
            vehicle.sensors.poll('electrics')
            speed = electrics.data['wheelspeed']

            host.send(socket, 'S', speed)            

            # Get ADAS sensors data
            if speed >= 8.333: # Speed for LiDAR
                vehicle.sensors.poll('state', 'timer')
                lidar_data_readonly = lidar.stream()
                pos = np.array([vehicle.state['pos'][0], vehicle.state['pos'][1] - 2.25, vehicle.state['pos'][2] + 0.6])
                direction = vehicle.state['dir']

                lidar_data = lidar_data_readonly.copy()[:np.where(lidar_data_readonly == 0)[0][0]]
                lidar_data = lidar_data.reshape((len(lidar_data) // 3, 3))

                transform = np.identity(4)
                transform[:3, 3] = -pos

                lidar_data = np.column_stack((lidar_data, np.ones(len(lidar_data))))
                lidar_data = np.dot(lidar_data, transform.T)[:, :3]

                lidar_data = lidar_data[np.dot(lidar_data, direction) >= 0]

                host.send(socket, 'L', timer['time'], direction[:2], lidar_data.tobytes())
                
            if second % 2 == 0:
                if speed >= 11.111: # Speed for camera
                    camera_data = camera.stream_colour(3686400)
                    camera_data = np.array(camera_data).reshape(height, width, 4)
                    camera_data = (0.299 * camera_data[:, :, 0] + 0.587 * camera_data[:, :, 1] + 0.114 * camera_data[:, :, 2]).astype(np.uint8)
                    camera_data = Image.fromarray(camera_data, 'L')

                    camera_data.save('cam.png', 'PNG')
                    img = open('cam.png', 'rb').read()

                    host.send(socket, 'C', img)
                elif speed <= 3.333: # Parking speed
                    vehicle.sensors.poll('electrics', 'timer')
                    park_data = np.array([uss_f.stream()[0], uss_fl.stream()[0], uss_fr.stream()[0], 
                            uss_r.stream()[0], uss_rl.stream()[0], uss_rr.stream()[0]], dtype=np.float32)
                    host.send(socket, 'P', timer['time'], electrics.data['gear'], park_data)
                else: # Blind spot detection
                    blind_data = [uss_left.stream(), uss_right.stream()]
                    host.send(socket, 'B', blind_data)

            bng.resume()
            second += 1

            # Note for blind spots: Use hystheresis to filter noise

    except Exception as e:
        print(e)
    finally:
        socket.close()
    exit_event.set()
    print('Exiting...')
    host.destroy(home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer)
    
def receiver(proc, ready_event, ready_back_event, exit_event):
    print('Waiting for sender...')
    ready_event.wait()
    print('Starting receiver...')
    home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer = host.init('sp0', False, False)

    print('Opening connection...')

    socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket.bind(('0.0.0.0', 4441 + proc))
    print('Waiting for connection...')
    socket.listen()
    conn, addr = socket.accept()
    print('Receiver operational...')
    ready_back_event.set()
    while True:
        if exit_event.is_set():
            break
        adas_response, response_type = host.recv(conn) # np.frombuffer? see from procesor

        if response_type == 'I':
            adas_response = np.frombuffer(adas_response, dtype=np.float_)
            vehicle.control(throttle=adas_response[0])
            vehicle.control(throttle=adas_response[1])
        elif response_type == 'B':
            adas_response = np.frombuffer(adas_response, dtype=np.bool_)
            if adas_response[0]:
                print('blind spot left')
            if adas_response[1]:
                print('blind spot right')

    print('Exiting...')
    host.destroy(home, bng, scenario, vehicle, camera, lidar, uss_f, uss_fl, uss_fr, uss_r, uss_rl, uss_rr, uss_left, uss_right, electrics, timer)

def main():
    ready_event = Event()
    ready_cam_event = Event()
    ready_lidar_event = Event()
    exit_event = Event()

    sender_proc = Process(target=sender, args=(ready_event, ready_cam_event, ready_lidar_event, exit_event))
    cam_proc = Process(target=receiver, args=(1, ready_event, ready_cam_event, exit_event))
    lidar_proc = Process(target=receiver, args=(2, ready_event, ready_lidar_event, exit_event))

    sender_proc.start()
    # receiver_proc.start()

    sender_proc.join()

if __name__ == '__main__':
    main()