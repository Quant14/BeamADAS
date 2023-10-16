import cv2

from lane_curve_and_offset import LaneCurve

lc = LaneCurve()

for i in range(0, 30):
    left, right, offset = lc.lane_pipeline(cv2.imread('.\\img' + str(i) + '.png', cv2.IMREAD_GRAYSCALE), i)
    print(str(i) + ':\nleft radius: ' + str(left) + '\nright radius: ' + str(right) + '\noffset: ' + str(offset))