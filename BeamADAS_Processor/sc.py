import numpy as np

def cam_speed_control(dist, speed, target):
    if speed > target:
        return 0, ((speed * speed - target * target) / (2 * dist)) / 100 % 1
    else:
        return 100, 0
    
def lidar_speed_control(dist, speed, target):
    dist -= speed * 2
    
    if target <= 0:
        target = 0
    if dist < 1:
        dist = 1

    brake = np.clip(((speed * speed - target * target) / (2 * dist)) / 100, 0.0, 1.0)
    
    if brake >= 0.3:
        return 0, brake
    elif brake >= 0.1:
        return 0, 0

    return 100, 0