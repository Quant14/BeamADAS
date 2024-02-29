import numpy as np

def cam_speed_control(dist, speed, target):
    if speed > target:
        return 0, np.clip(((speed * speed - target * target) / (2 * dist)) / 100, 0.0, 1.0)
    else:
        return 100, 0

def lidar_speed_control(dist, speed, target):
    dist -= target * 2

    if dist < 1:
        dist = 1

    brake = np.clip(((speed * speed - target * target) / (2 * dist)) / 100, 0.0, 1.0)

    if brake >= 0.3:
        return 0, brake
    elif brake >= 0.1:
        return 0, 0

    return 100, 0

def uss_speed_control(dist, speed):
    brake = np.clip(speed * speed / (2 * dist) / 100, 0.0, 1.0)
    if brake >= 75:
        return 0, brake

    return 100, 0