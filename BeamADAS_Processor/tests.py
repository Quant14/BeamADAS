import cv2

from lco import LaneCurve

lc = LaneCurve()
res = []

radius = lc.lane_pipeline(cv2.imread('./sp1/sample2/cam/img12.png', cv2.IMREAD_GRAYSCALE))
print('estimated radius: ' + str(radius))

radius = lc.lane_pipeline(cv2.imread('./sp1/sample4/cam/img12.png', cv2.IMREAD_GRAYSCALE))
print('estimated radius: ' + str(radius))
    
# print(time.time())
# a = 22.677478

# print(len(a.tobytes()))
# print(a.to_bytes(2))