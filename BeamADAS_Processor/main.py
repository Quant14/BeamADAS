import comm
from lco import LaneCurve
from od import ObjectDetect
import sc

import numpy as np
import multiprocessing as mp
import math
import struct
import traceback
import time

import matplotlib as plt

wait_timeout = 20

# MAIN PROCESS ---------------------------------
def main_process(init_event, quit_event, speed, cam, cam_event, lidar, lidar_size, veh_dir, cam_timestamp, timestamp, lidar_event, uss, uss_event, gear, blind, blind_event, cam_last_brake, lidar_last_brake):
    socket = comm.Comm(0)
    try:
        while not quit_event.is_set():
            # print('Waiting for message...')
            data_type, data_len, curr_time, curr_dir, curr_gear, data = socket.recv_data()

            if data_type != None and data != None:
                if data_type == b'S':
                    # print('Recv speed')
                    with speed.get_lock():
                        speed.value = struct.unpack('>f', data)[0]
                        with cam_last_brake.get_lock() and lidar_last_brake.get_lock():
                            if speed.value < 8.333:
                                cam_last_brake.value = 0
                                lidar_last_brake.value = 0
                            elif speed.value < 11.111:
                                lidar_last_brake.value = 0
                elif data_type == b'B':
                    # print('Recv blind')
                    with blind.get_lock():
                        blind[:] = data
                    blind_event.set()
                elif data_type == b'C':
                    # print('Recv cam')
                    with cam.get_lock():
                        cam[:] = data
                        cam_timestamp.value = curr_time
                    cam_event.set()
                elif data_type == b'L':
                    # print('Recv lidar')
                    with lidar.get_lock():
                        timestamp.value = curr_time
                        veh_dir[:] = curr_dir
                        lidar[:data_len] = data
                        lidar_size.value = data_len
                    lidar_event.set()
                elif data_type == b'P':
                    # print('Recv park')
                    with uss.get_lock():
                        uss[:] = data
                        timestamp.value = curr_time
                        gear.value = curr_gear
                    uss_event.set()
                elif data_type == b'I':
                    print('Recv init')
                    init_event.set()
                elif data_type == b'Q':
                    print('Recv quit')
                    quit_event.set()

    except Exception as e:
        traceback.print_exc()
    finally:
        socket.close()

# CAM PROCESS ----------------------------------
def cam_process(init_event, quit_event, speed, cam, timestamp, event, cam_last_brake, lidar_last_brake):
    init_event.wait()
    print('Cam starting')
    lc = LaneCurve()
    socket = comm.Comm(1)
    try:
        curr_time = 0.0
        while not quit_event.is_set():
            print('Cam: Waiting...')
            t = time.time()
            event.wait(timeout=wait_timeout)
            if time.time() - t >= wait_timeout:
                if quit_event.is_set():
                    break
                else:
                    continue
            event.clear()

            print('Cam: Processing...')
            with cam.get_lock():
                img = np.frombuffer(cam.get_obj(), dtype=np.uint8).reshape((720, 1280))
                curr_time = timestamp.value

            with speed.get_lock():
                curr_speed = speed.value

            radius = lc.lane_pipeline(img, curr_time)
            if radius != None:
                max_speed = math.sqrt(4.905 * radius) * (3.6 + 0.4) # type: ignore
                print(f'Cam: Max speed: {max_speed}')

                throttle, brake = sc.cam_speed_control(5, curr_speed, max_speed)
                print(f'Cam: Throttle: {throttle}, Brake: {brake}')

                with lidar_last_brake.get_lock():
                    if lidar_last_brake.value <= brake:
                        socket.send_data(b'I', np.array([throttle, brake], dtype=np.float32))

                with cam_last_brake.get_lock():
                    cam_last_brake.value = brake

    except Exception as e:
        traceback.print_exc()
    finally:
        socket.close()

# LIDAR PROCESS --------------------------------
def lidar_process(init_event, quit_event, speed, lidar, lidar_size, veh_dir, timestamp, lidar_event, cam_last_brake, lidar_last_brake):
    init_event.wait()
    print('Lidar starting')
    socket = comm.Comm(2)
    try:
        curr_speed = 0.0
        curr_dir = [0.0, 0.0]
        curr_time = 0.0
        curr_data = np.array([], dtype=np.float32)
        od = ObjectDetect()

        while not quit_event.is_set():
            # print('Lidar: Waiting...')
            t = time.time()
            lidar_event.wait(timeout=wait_timeout)
            if time.time() - t >= wait_timeout:
                if quit_event.is_set():
                    break
                else:
                    continue
            if quit_event.is_set():
                break
            lidar_event.clear()

            # print('Lidar: Processing...')
            with lidar.get_lock():
                curr_dir = veh_dir[:]
                curr_time = timestamp.value
                curr_data = np.frombuffer(lidar.get_obj(), dtype=np.float32, count=2400).reshape(800, 3)
                curr_data = curr_data[:(lidar_size.value // 12)]
            with speed.get_lock():
                curr_speed = speed.value

            matched_info, relevant_indices = od.lidar_pipeline(curr_data, curr_time, curr_speed, curr_dir)
            max_brake = 0.0
            min_throttle = 100.0
            if (relevant_indices is not None and matched_info is not None) and (len(relevant_indices) > 0 and len(matched_info) > 0):
                print('Lidar: Found risk objects')
                for i in relevant_indices:
                    dist, sp, pos, dir = matched_info[i]
                    throttle, brake = sc.lidar_speed_control(dist, curr_speed, curr_speed - sp)
                    if brake > max_brake:
                        max_brake = brake
                        min_throttle = throttle

            print(f'Lidar: Throttle: {min_throttle}, Brake: {max_brake}')
            with cam_last_brake.get_lock():
                if cam_last_brake.value <= max_brake or curr_speed < 11.111:
                    socket.send_data(b'I', np.array([min_throttle, max_brake], dtype=np.float32))

            with lidar_last_brake.get_lock():
                lidar_last_brake.value = max_brake

    except Exception as e:
        traceback.print_exc()
    finally:
        socket.close()

# USS PROCESS ----------------------------------
def uss_process(init_event, quit_event, uss, uss_event, timestamp, gear):
    init_event.wait()
    print('USS starting')
    socket = comm.Comm(3)
    try:
        curr_time = 0.0
        curr_gear = b''
        curr_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        rel_data = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        prev_data = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        prev_dir = 0
        prev_time = 0.0
        throttle = 100.0
        brake = 0.0

        while not quit_event.is_set():
            print('USS: Waiting...')
            t = time.time()
            uss_event.wait(timeout=wait_timeout)
            if time.time() - t >= wait_timeout:
                if quit_event.is_set():
                    break
                else:
                    continue
            uss_event.clear()

            print('USS: Processing...')
            with uss.get_lock():
                curr_time = timestamp.value
                curr_gear = gear.value
                curr_data = np.frombuffer(uss.get_obj(), dtype=np.float32, count=6)
                # print(f'Curr time: {curr_time}')
                # print(f'Curr gear: {curr_gear}')
                # print(f'Curr dtaa: {curr_data}')

            if curr_gear != b'N' and curr_gear != b'P':
                if curr_gear != b'R':
                    rel_data = curr_data[:3] - 0.1
                    if prev_dir != 1:
                        prev_dir = 1
                        prev_data[0] = -1.0
                else:
                    rel_data = curr_data[-3:]
                    if prev_dir != -1:
                        prev_dir = -1
                        prev_data[0] = -1.0

                # print(f'Rel data: {rel_data}')
                # print(f'Prev data: {prev_data}')

                d_time = curr_time - prev_time
                # print(f'Time: {d_time}')
                if prev_time != 0 and d_time <= 3:
                    if prev_data[0] != -1.0:
                        max_speed = 0.0
                        dist = 1000.0
                        rel_data = np.clip(rel_data, a_min=0.0, a_max=3)
                        for i in range(0, 3):
                            speed = (prev_data[i] - rel_data[i]) / d_time
                            if speed > max_speed:
                                max_speed = speed
                                dist = rel_data[i] - 0.2
                                break

                        # print(f'USS: Max closing speed: {max_speed}')
                        throttle, brake = sc.uss_speed_control(dist, max_speed)

            # print(f'USS: Throttle: {throttle}, Brake: {brake}')
            socket.send_data(b'I', np.array([throttle, brake], dtype=np.float32))

            prev_data = rel_data
            prev_time = curr_time
    except Exception as e:
        traceback.print_exc()
    finally:
        socket.close()

# BLIND PROCESS --------------------------------
def blind_process(init_event, quit_event, blind, blind_event):
    init_event.wait()
    print('Blind starting')
    socket = comm.Comm(4)
    try:
        curr_data = np.array(2, dtype=np.float32)

        while not quit_event.is_set():
            # print('Blind: Waiting...')
            t = time.time()
            blind_event.wait(timeout=wait_timeout)
            if time.time() - t >= wait_timeout:
                if quit_event.is_set():
                    break
                else:
                    continue
            blind_event.clear()

            # print('Blind: Processing...')
            with blind.get_lock():
                curr_data = np.frombuffer(blind.get_obj(), dtype=np.float32, count=2)

            # print(f'Left dist: {curr_data[0]}, Right dist: {curr_data[1]}')
            socket.send_data(b'B', np.array([curr_data[0] < 2, curr_data[1] < 2]))
    except Exception as e:
        traceback.print_exc()
    finally:
        socket.close()

# MAIN SETUP -----------------------------------
def main():
    try:
        speed = mp.Value('f', 0.0, lock=True)

        cam = mp.Array('B', 921600, lock=True)
        cam_timestamp = mp.Value('f', 0.0)
        cam_event = mp.Event()
        cam_last_brake = mp.Value('f', 0.0, lock=True)

        lidar = mp.Array('b', 9600, lock=True)
        lidar_size = mp.Value('i', 0)
        veh_dir = mp.Array('f', 2)
        timestamp = mp.Value('f', 0.0)
        lidar_event = mp.Event()
        lidar_last_brake = mp.Value('f', 0.0, lock=True)

        uss = mp.Array('B', 24, lock=True)
        uss_event = mp.Event()
        gear = mp.Value('c', b'N')

        blind = mp.Array('B', 8, lock=True)
        blind_event = mp.Event()

        init_event = mp.Event()
        quit_event = mp.Event()

        main_proc = mp.Process(target=main_process, args=(init_event, quit_event, speed, cam, cam_event, lidar, lidar_size, veh_dir, cam_timestamp, timestamp, lidar_event, uss, uss_event, gear, blind, blind_event, cam_last_brake, lidar_last_brake))
        cam_proc = mp.Process(target=cam_process, args=(init_event, quit_event, speed, cam, cam_timestamp, cam_event, cam_last_brake, lidar_last_brake))
        lidar_proc = mp.Process(target=lidar_process, args=(init_event, quit_event, speed, lidar, lidar_size, veh_dir, timestamp, lidar_event, cam_last_brake, lidar_last_brake))
        uss_proc = mp.Process(target=uss_process, args=(init_event, quit_event, uss, uss_event, timestamp, gear))
        blind_proc = mp.Process(target=blind_process, args=(init_event, quit_event, blind, blind_event))

        main_proc.start()
        # cam_proc.start()
        lidar_proc.start()
        # uss_proc.start()
        # blind_proc.start()

        print('All systems started')

        main_proc.join()
        # cam_proc.join()
        lidar_proc.join()
        # uss_proc.join()
        # blind_proc.join()

        print('All systems shut down')
    except Exception as e:
        traceback.print_exc()
    finally:
        print('done')

if __name__ == '__main__':
    main()