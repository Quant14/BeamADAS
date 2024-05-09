import numpy as np
import sympy as sp
from sklearn.cluster import DBSCAN
from scipy.optimize import linear_sum_assignment

class ObjectDetect:
    def __init__(self):
        self.centroids_prev = np.array([])
        self.timestamp_prev = 0.0

    def find_clusters(self, lidar_data):
        distance_thresholds = [(2.5, 25.0), (25.0, 50.0), (50.0, 100.0)]

        cluster_data = []

        for i, threshold in enumerate(distance_thresholds):
            low_lim, high_lim = threshold
            linalg = np.linalg.norm(lidar_data[:, :2], axis=1)
            segment_criteria = (linalg >= low_lim) & (linalg < high_lim)
            segment_data = lidar_data[segment_criteria]
            if len(segment_data) > 0:
                if i == 0:
                    dbscan = DBSCAN(eps=0.6, min_samples=25, n_jobs=1, algorithm='ball_tree') # near
                elif i == 1:
                    dbscan = DBSCAN(eps=1.2, min_samples=15, n_jobs=1, algorithm='ball_tree') # middle
                else:
                    dbscan = DBSCAN(eps=1.3, min_samples=10, n_jobs=1, algorithm='ball_tree') # far

                clusters = dbscan.fit_predict(segment_data)
                for cluster_id in np.unique(clusters[clusters != -1]):
                    cluster_points = segment_data[clusters == cluster_id]
                    cluster_data.append(cluster_points) # Points only

                lidar_data = lidar_data[np.logical_not(segment_criteria)]

        return cluster_data

    def match_and_track(self, cluster_data, elapsed, speed):
        centroids = np.array([np.mean(obj, axis=0) for obj in cluster_data])

        thresh = elapsed * speed

        distances = np.linalg.norm(self.centroids_prev[:, np.newaxis, :] - centroids, axis=2)
        filtered_dist = distances[np.where(distances[:, 0] < thresh)[0]]
        row_ind, col_ind = linear_sum_assignment(filtered_dist)
        matched_clusters = np.column_stack((row_ind, col_ind))

        matched_info = [] # [[distance, speed, position, direction], [...], ...]

        for i, j in matched_clusters:
            prev_cluster = self.centroids_prev[i][:2]
            curr_cluster = centroids[j][:2]
            # print(f'CLUSTER POS: {centroids[j]}')

            movement_speed = distances[i, j] / elapsed

            if movement_speed < 120:
                movement_vector = curr_cluster - prev_cluster
                matched_info.append((np.linalg.norm(curr_cluster),
                                    movement_speed,
                                    curr_cluster,
                                    movement_vector[:2]))

        self.centroids_prev = centroids

        return matched_info

    def analyze_relevancy(self, matched_info, veh_dir, accX):
        x, y = sp.symbols('x y')

        cubic_equation = sp.Eq(y, accX * x**3)
        theta = 1.5708 - np.arctan2(veh_dir[1], veh_dir[0])

        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), -np.cos(theta)]], dtype=np.float32)

        relevant_indices = np.array([], dtype=np.uint8)
        i = 0

        for dist, speed, pos, dir in matched_info:
            if np.dot(dir / np.linalg.norm(dir), veh_dir) > -0.7:
            #    print(f'Veh dir: {veh_dir}')
               continue

            pivoted_pos = np.dot(rotation_matrix, pos)
            pivoted_dir = np.dot(rotation_matrix, dir)

            A = -pivoted_dir[1]
            B = pivoted_dir[0]
            C = -B * pivoted_pos[1] + dir[1] * pivoted_pos[0] # optimized A1 to eliminate double negative

            line_equation = sp.Eq(0, A * x + B * y + C)

            intersection_equation = cubic_equation.subs(y, (-A * x - C) / B)

            intersection_x = sp.solve(intersection_equation, x)

            intersection_points = [(x_val, cubic_equation.subs(x, x_val)) for x_val in intersection_x]

            for intersection in intersection_points:
                if np.linalg.norm(intersection) < 1.5:
                    relevant_indices = np.append(relevant_indices, i)
                    break

            i += 1

        return relevant_indices

    def lidar_pipeline(self, lidar_data, timestamp, speed, dir, accX):
        cluster_data = self.find_clusters(lidar_data)

        if len(cluster_data) > 0:
            if self.timestamp_prev != 0.0 and timestamp - self.timestamp_prev <= 1:
                matched_info = self.match_and_track(cluster_data, timestamp - self.timestamp_prev, speed)
                relevant_indices = self.analyze_relevancy(matched_info, dir, accX)
                self.timestamp_prev = timestamp
                return matched_info, relevant_indices
            else:
                self.centroids_prev = np.array([np.mean(obj, axis=0) for obj in cluster_data])
                self.timestamp_prev = timestamp
        return None, None