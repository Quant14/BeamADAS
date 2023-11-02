import cv2
# import matplotlib.pyplot as plt
import time

from lane_curve_and_offset import LaneCurve

lc = LaneCurve()
res = []

print(time.time())
for i in range(0, 30):
    radius, offset = lc.lane_pipeline(cv2.imread('./img' + str(i) + '.png', cv2.IMREAD_GRAYSCALE), i)
    print(str(i) + ':\nestimated radius: ' + str(radius) + '\noffset: ' + str(offset))
    
print(time.time())
a = 22.677478

# print(len(a.tobytes()))
# print(a.to_bytes(2))