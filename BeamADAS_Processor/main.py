import comm
from lco import LaneCurve
from od import ObjectDetect
import sc

import cv2
import numpy as np
import multiprocessing as mp
import math
import struct

# MAIN PROCESS ---------------------------------
def main_process(init_event, quit_event, speed, cam, cam_size, cam_event, lidar, lidar_size, veh_dir, timestamp, lidar_event):
    socket = comm.Comm(0)
    try:
        while True:
            data_type, data_len, curr_time, curr_dir, gear, data = socket.recv_data()

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
                        veh_dir[:] = curr_dir
                        lidar[:data_len] = data
                        lidar_size.Value = data_len
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
def cam_process(init_event, quit_event, speed, cam, size, timestamp, event, cam_last_brake, lidar_last_brake):
    init_event.wait()
    lc = LaneCurve()
    socket = comm.Comm(1)
    try:
        curr_time = 0.0
        while not quit_event.is_set():
            event.wait()
            event.clear()

            with cam.get_lock():
                img = np.frombuffer(cam.get_obj(), dtype=np.uint8, count=size.Value)
                img = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)
                curr_time = timestamp.Value

            radius = lc.lane_pipeline(img, curr_time)
            if radius != None:
                max_speed = math.sqrt(4.905 * radius) * (3.6 + 0.4) # type: ignore

                with speed.get_lock():
                    throttle, brake = sc.cam_speed_control(5, speed.Value, max_speed)

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
def lidar_process(init_event, quit_event, speed, lidar, lidar_size, veh_dir, timestamp, lidar_event, cam_last_brake, lidar_last_brake):
    init_event.wait()
    socket = comm.Comm(2)
    try:
        curr_speed = 0.0
        curr_dir = [0.0, 0.0]
        curr_time = 0.0
        curr_data = np.array([], dtype=np.float32)
        od = ObjectDetect()

        while not quit_event.is_set():
            lidar_event.wait()
            lidar_event.clear()
            with lidar.get_lock():
                curr_dir = veh_dir[:]
                curr_time = timestamp.Value
                curr_data = np.frombuffer(lidar.get_obj(), dtype=np.float32, count=lidar_size.Value).reshape(lidar_size.Value // 3, 3)

            with speed.get_lock():
                curr_speed = speed.Value

            matched_info, relevant_indices = od.lidar_pipeline(curr_data, curr_time, curr_speed, curr_dir)
            if relevant_indices != None and matched_info != None:
                throttle = 0.0
                brake = 0.0
                max_brake = 0.0
                for i in relevant_indices:
                    dist, sp, pos, dir = matched_info[i]
                    throttle, brake = sc.lidar_speed_control(dist, curr_speed, curr_speed - sp)
                    if brake > max_brake:
                        max_brake = brake

                with cam_last_brake.get_lock():
                    if cam_last_brake < brake:
                        socket.send_data(b'C', np.array([throttle, brake], dtype=np.float32))

                with lidar_last_brake.get_lock():
                    lidar_last_brake.Value = brake

    except Exception as e:
        print(e)
    finally:
        socket.close()

def uss_process(init_event, quit_event, uss, uss_event, timestamp, gear):
    init_event.wait()
    socket = comm.Comm(3)
    try:
        curr_time = 0.0
        curr_data = np.array(6, dtype=np.float32)
        rel_data = np.array(3, dtype=np.float32)
        prev_data = np.array(3, dtype=np.float32)
        prev_dir = 0
        prev_time = 0.0

        while not quit_event.is_set():
            uss_event.wait()
            uss_event.clear()
            with uss.get_lock():
                curr_time = timestamp.Value
                curr_data = np.frombuffer(uss.get_obj(), dtype=np.float32, count=6)

            if gear != 'N' and gear != 'P':
                if gear != 'R':
                    rel_data = curr_data[:3]
                    if prev_dir != 1:
                        prev_dir = 1
                        prev_data[0] = -1.0
                else:
                    rel_data = curr_data[-3:]
                    if prev_dir != -1:
                        prev_dir = -1
                        prev_data[0] = -1.0

            d_time = curr_time - prev_time
            if prev_time != 0 and d_time <= 3:
                if prev_data[0] != -1:
                    max_speed = 0.0
                    dist = 0.0
                    rel_data = np.clip(rel_data, a_min=None, a_max=5) # NOTE: CHECK IF 5 IS REAL CAP OF SENSOR
                    for i in range(0, 3):
                        speed = (prev_data[i] - rel_data[i]) / d_time
                        if speed > max_speed:
                            max_speed = speed
                            dist = rel_data[i] - 0.2
                            break

                    throttle, brake = sc.uss_speed_control(dist, max_speed)

                    socket.send_data(b'C', np.array([throttle, brake], dtype=np.float32))

            prev_data = rel_data
            prev_time = curr_time
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
        cam_timestamp = mp.Value('f', 0.0)
        cam_event = mp.Event()
        cam_last_brake = mp.Value('f', 0.0, lock=True)

        lidar = mp.Array('f', 2400, lock=True)
        lidar_size = mp.Value('I', 0)
        veh_dir = mp.Array('f', 2)
        timestamp = mp.Value('f', 0.0)
        lidar_event = mp.Event()
        lidar_last_brake = mp.Value('f', 0.0, lock=True)

        uss = mp.Array('f', 8, lock=True)
        uss_event = mp.Event()

        init_event = mp.Event()
        quit_event = mp.Event()

        main_proc = mp.Process(target=main_process, args=(init_event, quit_event, speed, cam, cam_size, cam_event, lidar, lidar_size, veh_dir, cam_timestamp, timestamp, lidar_event, uss, uss_event))
        cam_proc = mp.Process(target=cam_process, args=(init_event, quit_event, speed, cam, cam_size, cam_timestamp, cam_event, cam_last_brake, lidar_last_brake))
        lidar_proc = mp.Process(target=lidar_process, args=(init_event, quit_event, speed, lidar, lidar_size, veh_dir, timestamp, lidar_event, cam_last_brake, lidar_last_brake))
        uss_proc = mp.Process(target=uss_process, args=(init_event, quit_event, uss, uss_event, speed, timestamp))

        main_proc.start()
        cam_proc.start()
        lidar_proc.start()
        uss_proc.start()

        print('All systems started')

        main_proc.join()
        cam_proc.join()
        lidar_proc.join()
        uss_proc.join()

        print('All systems shut down')
    except Exception as e:
        print(e)
    finally:
        print('done')

if __name__ == '__main__':
    mp.freeze_support() # NOTE: Remove for Raspberry
    main()