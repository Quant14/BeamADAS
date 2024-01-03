import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import DBSCAN

# import comm

# socket = comm.Comm()

# try:
#     data = socket.recv_data()
#     if data == None: exit()
#     # data = np.frombuffer(data, dtype=np.float32).reshape((800, 3))
#     # print(data)
#     # with open('socket.png', 'wb') as file:
#     #     file.write(data)

#     socket.send_data(b'')
# except Exception as e:
#     print(e)
# finally:
#     socket.close()

# exit()
# print('a')
# size = struct.unpack('H', ser.read(2))[0]
# print('b')
# data = struct.unpack(f'{size}f', ser.read(4 * size))
# print('c')

# lidar_data = np.array(data).reshape((size // 3, 3))

class ObjectDetect:
    def __init__(self):
        self.lidar_data_prev = np.zeros((800, 3))
        self.timestamp_prev = 0.0

    def find_clusters(self, lidar_data):
        distance_thresholds = [(2.5, 25.0), (25.0, 50.0), (50.0, 100.0)]

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
                    dbscan = DBSCAN(eps=1, min_samples=12, n_jobs=1, algorithm='ball_tree') # middle
                else:
                    dbscan = DBSCAN(eps=1.5, min_samples=8, n_jobs=1, algorithm='ball_tree') # far

                clusters = dbscan.fit_predict(segment_data)
                for cluster_id in np.unique(clusters[clusters != -1]):
                    cluster_points = segment_data[clusters == cluster_id]
                    cluster_data.append((i, cluster_id, cluster_points))

                lidar_data = lidar_data[np.logical_not(segment_criteria)]

        return cluster_data

    def analyze_clusters(self, cluster_data):
        clusters_info = []
        for cluster_info in cluster_data:
            j, cluster_id, cluster_points = cluster_info
            distances = np.linalg.norm(cluster_points - cluster_points[:, np.newaxis], axis=2)
            min_distance_idx = np.unravel_index(np.argmin(distances), distances.shape)
            closest_point = cluster_points[min_distance_idx[0]]
            distance = np.linalg.norm(closest_point)

            clusters_info.append((j, cluster_id, distance))

            print(f"Segment {j}, Cluster {cluster_id}: Closest point {closest_point}, Distance: {distance} meters")

od = ObjectDetect()

for j in range(0, 5):
    print(j * 3)
    path = f'sp2/sample2/lidar/pc{j * 3}.txt'
    lidar_data = np.genfromtxt(path, delimiter=' ')
    t = time.time()
    cluster_data = od.find_clusters(lidar_data)
    od.analyze_clusters(cluster_data)
    print(time.time() - t)
    lidar_data_original = lidar_data # Plotting

    # Plotting
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