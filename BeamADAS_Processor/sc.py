import numpy

def speed_control(dist, speed, target):
    if target <= speed:
        if dist == 0:
            dist = 0.0001
        return 0, 100 * (target * target - speed * speed) / (2 * dist)