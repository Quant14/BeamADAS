import comm
from lco import LaneCurve
from sc import speed_control

import cv2
import numpy as np
import multiprocessing as mp
import time
import math
import struct

# THINK OF WAY TO TRIGGER FINAL ADAS SIGNAL 100 THROTTLE 0 BRAKE

# MAIN PROCESS ---------------------------------
def main_process(init_event, quit_event, speed, cam, cam_size, cam_event, lidar, veh_dir, timestamp, lidar_event):
    socket = comm.Comm(0)
    try:
        while True:
            data_type, data_len, curr_time, direction, data = socket.recv_data()

            if data_type != None and data != None:
                if data_type == 'S':
                    with speed.get_lock():
                        speed.Value = struct.unpack('>f', data)
                elif data_type == 'C':
                    with cam.get_lock():
                        cam[:data_len] = data
                        cam_size.Value = data_len
                    cam_event.set()
                elif data_type == 'L':
                    with lidar.get_lock():
                        timestamp.Value = curr_time
                        veh_dir[:] = direction
                        lidar[:] = data
                    lidar_event.set()
                elif data_type == 'I':
                    init_event.set()
                elif data_type == 'Q':
                    quit_event.set()        

    except Exception as e:
        print(e)
    finally:
        socket.close()

# CAM PROCESS ----------------------------------
def cam_process(init_event, quit_event, speed, cam, size, event, cam_last_brake, lidar_last_brake):
    init_event.wait()
    lc = LaneCurve()
    socket = comm.Comm(1)
    try:
        while not quit_event.is_set():
            event.wait()
            event.clear()

            with cam.get_lock():
                img = np.frombuffer(cam, dtype=np.uint8, count=size.Value)
                img = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)

            radius, pos = lc.lane_pipeline(img)
            max_speed = math.sqrt(4.905 * radius) * 3.6

            with speed.get_lock():
                throttle, brake = speed_control(5, speed.Value, max_speed)

            with lidar_last_brake.get_lock():
                if lidar_last_brake < brake:
                    socket.send_data(b'C', np.array([throttle, brake], dtype=np.float32))
            
            with cam_last_brake.get_lock():
                cam_last_brake.Value = brake

    except Exception as e:
        print(e)
    finally:
        socket.close()

# LIDAR PROCESS --------------------------------
def lidar_process(init_event, quit_event, speed, lidar, veh_dir, timestamp, lidar_event, cam_last_brake, lidar_last_brake):
    init_event.wait()
    socket = comm.Comm(2)
    try:
        curr_dir = [0.0, 0.0]
        curr_time = 0.0
        curr_data = np.array([], dtype=np.float32)
        while not quit_event.is_set():
            lidar_event.wait()
            lidar_event.clear()
            with lidar.get_lock():
                curr_dir = veh_dir[:]
                curr_time = timestamp.Value
                curr_data = np.frombuffer(lidar.get_obj(), dtype=np.float32).reshape(800, 3)
                print(f'sub: {curr_data}')

            # find important object
            # calc target speed
            # calc distance to target gap to object\
            with speed.get_lock():
                throttle, brake = speed_control(dist, speed, target)
            if brake >= 0.5:
                with cam_last_brake.get_lock():
                    if cam_last_brake < brake:
                        socket.send_data(b'C', np.array([throttle, brake], dtype=np.float32))

                with lidar_last_brake.get_lock():
                    lidar_last_brake.Value = brake
            # speed_control() -> don't forget speed lock

    except Exception as e:
        print(e)
    finally:
        socket.close()

# MAIN SETUP -----------------------------------
def main():
    try:
        speed = mp.Value('f', 0.0, lock=True)

        cam = mp.Array('B', 1048576, lock=True)
        cam_size = mp.Value('I', 0)
        cam_event = mp.Event()
        cam_last_brake = mp.Value('f', 0.0, lock=True)

        lidar = mp.Array('f', 2400, lock=True)
        veh_dir = mp.Array('f', 2)
        timestamp = mp.Value('f', 0.0, lock=True)
        lidar_event = mp.Event()
        lidar_last_brake = mp.Value('f', 0.0, lock=True)

        uss = mp.Array('f', 8, lock=True)
        uss_event = mp.Event()

        init_event = mp.Event()
        quit_event = mp.Event()

        main_proc = mp.Process(target=main_process, args=(init_event, quit_event, speed, cam, cam_size, cam_event, lidar, veh_dir, timestamp, lidar_event, uss, uss_event))
        cam_proc = mp.Process(target=cam_process, args=(init_event, quit_event, speed, cam, cam_size, cam_event, cam_last_brake, lidar_last_brake))
        lidar_proc = mp.Process(target=lidar_process, args=(init_event, quit_event, speed, lidar, veh_dir, timestamp, lidar_event, cam_last_brake, lidar_last_brake))

        main_proc.start()
        cam_proc.start()
        lidar_proc.start()

        main_proc.join()
        cam_proc.join()
        lidar_proc.join()
    except Exception as e:
        print(e)
    finally:
        print('done')

if __name__ == '__main__':
    mp.freeze_support() # Remove for Raspberry
    main()