import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize_scalar 


def create_spline(points):
    return CubicSpline(np.arange(len(points)), points)

def distance(midline_point, boundary_spline):

    # Use a numerical optimization method to find the minimum distance along the spline
    t_closest = minimize_scalar(lambda t: np.linalg.norm(midline_point-boundary_spline(t)), bounds=(0, len(boundary_spline.c) - 1), method='bounded').x

    return np.linalg.norm(midline_point- boundary_spline(t_closest))

def find_distances(midline, boundary):
    boundary_spline = create_spline(boundary)
    distances_midline = [distance(point, boundary_spline) for point in midline]

    return distances_midline

# Example usage
midline =   np.array([(1, 2), (3, 4), (5, 6)])
boundary =  np.array([(0, 0), (2, 2), (4, 4), (5,5)])
boundary2 = np.array([(3, 0), (6, 3), (9, 6)])


distances_midline = find_distances(midline, boundary)

print("Distances to Midline:", distances_midline)

distances_midline = find_distances(midline, boundary2)

print("Distances to Midline:", distances_midline)