import numpy as np
import json, sys

def find_nearest_neighbor(point, points_array):
    distances = np.linalg.norm(points_array - point, axis=1)
    nearest_index = np.argmin(distances)
    nearest_neighbor = points_array[nearest_index]
    return nearest_neighbor



d = json.load(open(sys.argv[1]))


b1 = np.array(d["yellow"])
b2 = np.array(d["blue"])


nearest_neighbors = np.array([[point, find_nearest_neighbor(point, b2)] for point in b1])



print("Nearest neighbor:", nearest_neighbors)

centers = np.sum(nearest_neighbors, axis=1) / 2

# Calculate the distances
distances_point1 = np.linalg.norm(centers - nearest_neighbors[:, 0, :], axis=1).reshape(-1, 1)
distances_point2 = np.linalg.norm(centers - nearest_neighbors[:, 1, :], axis=1).reshape(-1, 1)
exp = np.concatenate((centers, distances_point1, distances_point2), axis=1)
print("exp", exp)