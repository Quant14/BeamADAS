import cv2
import time

from lco import LaneCurve

import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("169.254.154.43", 4442))

# lc = LaneCurve()
# res = []

# for i in range(0, 3, 3):
#     radius = lc.lane_pipeline(cv2.imread(f'./sp1/sample2/cam/img{i}.png', cv2.IMREAD_GRAYSCALE), 1.0)
#     print('estimated radius: ' + str(radius))
# time.sleep(1)

# print(time.time())
# a = 22.677478

# print(len(a.tobytes()))
# print(a.to_bytes(2))