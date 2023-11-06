import serial
import numpy as np
import struct
import time
from sklearn.cluster import DBSCAN

# ser = serial.Serial('/dev/ttyS0', baudrate=115200)

# # Check communication
# print('Waiting for connection...')
# # ser.timeout = 60
# if ser.readline() == b'Pi check connection\n':
#     # ser.timeout = 5
#     ser.write(b'Host check connection\n')
#     if ser.readline() == b'OK\n':
#         print('OK')
#         # ser.timeout = None

# print('a')
# size = struct.unpack('H', ser.read(2))[0]
# print('b')
# data = struct.unpack(f'{size}f', ser.read(4 * size))
# print('c')

# lidar_data = np.array(data).reshape((size // 3, 3))

lidar_data = np.genfromtxt('lidar.txt', delimiter=' ')

# Do work
dbscan = DBSCAN(eps=1, min_samples=3, n_jobs=1, algorithm='ball_tree')

t = time.time()
clusters = dbscan.fit_predict(lidar_data)

valid_clusters = []
for cluster_id in np.unique(clusters):
    if cluster_id == -1:
        continue

    cluster_points = lidar_data[clusters == cluster_id]

    distances = np.linalg.norm(cluster_points - cluster_points[:, np.newaxis], axis=2)
    min_distance_idx = np.unravel_index(np.argmin(distances), distances.shape)
    closest_point = cluster_points[min_distance_idx[0]]

    distance = np.linalg.norm(closest_point)

    valid_clusters.append((cluster_id, closest_point, distance))

valid_clusters = sorted(valid_clusters, key=lambda x: x[2])

print(time.time() - t)

for cluster_id, closest_point, distance in valid_clusters:
    print(f"Cluster {cluster_id}: Closest point {closest_point}, Distance: {distance} meters")
# ser.close()