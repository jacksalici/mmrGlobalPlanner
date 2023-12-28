import numpy as np

def find_nearest_neighbor(point, points_array):
    distances = np.linalg.norm(points_array - point, axis=1)
    nearest_index = np.argmin(distances)
    nearest_neighbor = points_array[nearest_index]
    return nearest_neighbor

import json, sys
d = json.load(open(sys.argv[1]))
b1 = np.array(d["yellow"])
b2 = d["blue"]
nearest_neighbors = np.array([[point, find_nearest_neighbor(point, b2)] for point in b1])

print("Nearest neighbor:", nearest_neighbors)