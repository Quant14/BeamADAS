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

for j in range(0, 5):
    print(j * 3)
    path = f'sp2/sample4/lidar/pc{j * 3}.txt'
    # path = 'sp2/sample3/lidar/pc0.txt'
    lidar_data = np.genfromtxt(path, delimiter=' ')
    lidar_data_original = lidar_data
    distance_thresholds = [(2.5, 25.0), (25.0, 60.0), (60.0, 100.0)]

    # Do work
    cluster_data = []

    for i, threshold in enumerate(distance_thresholds):
        low_lim, high_lim = threshold
        linalg = np.linalg.norm(lidar_data[:, :2], axis=1)
        segment_criteria = (linalg >= low_lim) & (linalg < high_lim)
        segment_data = lidar_data[segment_criteria]
        if len(segment_data) > 0:
            if i == 0:
                dbscan = DBSCAN(eps=0.5, min_samples=30, n_jobs=1, algorithm='ball_tree') # near
            elif i == 1:
                dbscan = DBSCAN(eps=1, min_samples=15, n_jobs=1, algorithm='ball_tree') # middle
            else:
                dbscan = DBSCAN(eps=1.5, min_samples=8, n_jobs=1, algorithm='ball_tree') # far

            clusters = dbscan.fit_predict(segment_data)
            for cluster_id in np.unique(clusters[clusters != -1]):
                cluster_points = segment_data[clusters == cluster_id]
                cluster_data.append((i, cluster_id, cluster_points))

            lidar_data = lidar_data[np.logical_not(segment_criteria)]

    for cluster_info in cluster_data:
        j, cluster_id, cluster_points = cluster_info
        distances = np.linalg.norm(cluster_points - cluster_points[:, np.newaxis], axis=2)
        min_distance_idx = np.unravel_index(np.argmin(distances), distances.shape)
        closest_point = cluster_points[min_distance_idx[0]]
        distance = np.linalg.norm(closest_point)

        print(f"Segment {j}, Cluster {cluster_id}: Closest point {closest_point}, Distance: {distance} meters")

    x = lidar_data_original[:, 0]
    y = lidar_data_original[:, 1]
    z = lidar_data_original[:, 2]

    plot = plt.figure().add_subplot(111, projection='3d')
    plot.scatter(x, y, z, alpha=0.1)

    for cluster_info in cluster_data:
        j, cluster_id, cluster_points = cluster_info
        plot.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2], alpha=1)

    plot.axis('equal')
    plt.show()
# ser.close()