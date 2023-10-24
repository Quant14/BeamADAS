import cv2
import matplotlib.pyplot as plt

from lane_curve_and_offset import LaneCurve

lc = LaneCurve()
res = []

for i in range(0, 30):
    radius, offset = lc.lane_pipeline(cv2.imread('.\\img' + str(i) + '.png', cv2.IMREAD_GRAYSCALE), i)
    res.append(radius)
    print(str(i) + ':\nestimated radius: ' + str(radius) + '\noffset: ' + str(offset))

plt.plot(res)
plt.show()