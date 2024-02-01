import cv2
import time

from lco import LaneCurve

lc = LaneCurve()
res = []

for i in range(0, 15, 3):
    radius = lc.lane_pipeline(cv2.imread(f'./sp1/sample3/cam/img{i}.png', cv2.IMREAD_GRAYSCALE), 1.0)
    print('estimated radius: ' + str(radius))
    time.sleep(1)

# print(time.time())
# a = 22.677478

# print(len(a.tobytes()))
# print(a.to_bytes(2))