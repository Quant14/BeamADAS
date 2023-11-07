import serial
import numpy as np
import struct
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
lidar_data_original = lidar_data
distance_thresholds = [(0.0, 50.0), (50.0, 100.0), (100.0, 150.0)]

# Do work
cluster_data = []

for i, threshold in enumerate(distance_thresholds):
    low_lim, high_lim = threshold
    linalg = np.linalg.norm(lidar_data[:, :2], axis=1)
    segment_criteria = (linalg >= low_lim) & (linalg < high_lim)
    segment_data = lidar_data[segment_criteria]
    
    dbscan = 0
    if i == 0:
        dbscan = DBSCAN(eps=0.4, min_samples=40, n_jobs=1, algorithm='ball_tree')
    elif i == 1:
        dbscan = DBSCAN(eps=0.7, min_samples=10, n_jobs=1, algorithm='ball_tree')
    else:
        dbscan = DBSCAN(eps=1, min_samples=3, n_jobs=1, algorithm='ball_tree')
    clusters = dbscan.fit_predict(segment_data)

    cluster_data.append(segment_data[clusters != -1])

    lidar_data = lidar_data[np.logical_not(segment_criteria)]

valid_clusters = []

# for i, cluster_points in enumerate(cluster_data):
#     distances = np.linalg.norm(cluster_points - cluster_points[:, np.newaxis], axis=2)
#     min_distance_idx = np.unravel_index(np.argmin(distances), distances.shape)
#     closest_point = cluster_points[min_distance_idx[0]]

#     distance = np.linalg.norm(closest_point)

#     valid_clusters.append((i, closest_point, distance))

# valid_clusters = sorted(valid_clusters, key=lambda x: x[2])

x = lidar_data_original[:, 0]
y = lidar_data_original[:, 1]
z = lidar_data_original[:, 2]

plot = plt.figure().add_subplot(111, projection='3d')
plot.scatter(x, y, z, alpha=0.1)

for cluster_points in cluster_data:
    plot.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2], alpha=1)
    
plt.show()

# for cluster_id, closest_point, distance in valid_clusters:
#     print(f"Cluster {cluster_id}: Closest point {closest_point}, Distance: {distance} meters")
# ser.close()