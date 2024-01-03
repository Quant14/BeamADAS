# import numpy

def speed_control(dist, speed, target):
    if speed > target:
        if dist < 1:
            dist = 1
        return 0, ((speed * speed - target * target) / (2 * dist)) / 100 % 1
    else:
        return 100, 0