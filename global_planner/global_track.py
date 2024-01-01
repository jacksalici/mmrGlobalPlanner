import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize_scalar 

import numpy as np
from enum import Enum

class Track():
    lines = Enum('Lines', ['TRACK', 'YELLOW', 'BLUE'])
    __points = {}
    __reftrack = None

    def add_line(self, line: lines, points: list):
        self.__points[line] = np.array(points, dtype=np.float)

    def has_boundaries(self) -> bool:
        return self.lines.YELLOW in self.__points and self.lines.BLUE in self.__points

    def __create_spline(self, points):
        return CubicSpline(np.arange(len(points)), points)

    def __distance(self, midline_point, boundary_spline):
        # Use a numerical optimization method to find the minimum distance along the spline
        t_closest = minimize_scalar(lambda t: np.linalg.norm(midline_point-boundary_spline(t)), bounds=(0, len(boundary_spline.c) - 1), method='bounded').x
        return np.linalg.norm(midline_point- boundary_spline(t_closest))

    def __find_distances(self, midline, boundary):
        boundary_spline = self.__create_spline(boundary)
        distances_midline = [self.__distance(point, boundary_spline) for point in midline]

        return np.array(distances_midline)
    
    def create_reftrack(self):
        w_l = self.__find_distances(self.__points[self.lines.TRACK], self.__points[self.lines.BLUE]).reshape(-1,1)
        w_r = self.__find_distances(self.__points[self.lines.TRACK], self.__points[self.lines.YELLOW]).reshape(-1,1)

        self.__reftrack = np.concatenate((self.__points[self.lines.TRACK], w_r, w_l), axis=-1)

        print(self.__reftrack)

    def is_reftrack_created(self):
        return self.__reftrack != None

# for testing

def main():


    # Example usage
    midline = np.array(  [[1, 2], [3, 4], [5, 6]])
    boundary = np.array( [[0, 0], [2, 2], [4, 4], [5,5]])
    boundary2 = np.array([[3, 0], [6, 3], [9, 6]])


    
    t = Track()

    t.add_line(points=midline, line=t.lines.TRACK)
    t.add_line(points=boundary2, line=t.lines.BLUE)
    t.add_line(points=boundary, line=t.lines.YELLOW)

    if t.has_boundaries() and not t.is_reftrack_created():
        t.create_reftrack()


if __name__ == "__main__":
    main()