import comm

import numpy as np
import multiprocessing as mp
import time

def main_process(cam, cam_event, lidar, lidar_event):
    socket = comm.Comm()
    try:
        while True:
            socket.recv_data()

            with lidar.get_lock():
                lidar[:] = np.random.rand(2400)
                print(f'main: {lidar[:]}')
            lidar_event.set()



            with cam.get_lock():
                cam[:len()]
            time.sleep(3)
    except Exception as e:
        print(e)
    finally:
        socket.close()

def cam_process(cam, cam_event):
    while True:
        print('cam')

def lidar_process(lidar, lidar_event):
    for i in range(0, 4):
        lidar_event.wait()
        with lidar.get_lock():
            curr_data = np.frombuffer(lidar.get_obj(), dtype=np.float32).reshape(800, 3)
            print(f'sub: {curr_data}')
        lidar_event.clear()

def main():
    try:
        cam = mp.Array('B', 1048576, lock=True)
        cam_size = mp.Value('I', 0)
        cam_event = mp.Event()

        lidar = mp.Array('f', 2400, lock=True)
        lidar_event = mp.Event()

        main_proc = mp.Process(target=main_process, args=(cam, cam_event, lidar, lidar_event))
        cam_proc = mp.Process(target=cam_process, args=(cam, cam_size, cam_event))
        lidar_proc = mp.Process(target=lidar_process, args=(lidar, lidar_event))

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
    mp.freeze_support() # Remove for Linux later builds
    main()