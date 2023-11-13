import cv2
import time

from lco import LaneCurve

lc = LaneCurve()
res = []

print(time.time())
for i in range(0, 30):
    radius, offset = lc.lane_pipeline(cv2.imread(f'./BeamADAS_Processor/cam/img{i}.png', cv2.IMREAD_GRAYSCALE), i)
    print(str(i) + ':\nestimated radius: ' + str(radius) + '\noffset: ' + str(offset))
    
print(time.time())
