import numpy as np
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize_scalar 
import matplotlib.pyplot as plt
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
        tck, u = splprep(points.T, per=True)
        return tck

    def __distance(self, midline_point, boundary_spline):
        distance_function = lambda t: np.linalg.norm(midline_point - np.array(splev(t, boundary_spline), dtype=np.float))

        # Use minimize_scalar to find the minimum distance along the spline
        result = minimize_scalar(distance_function, bounds=(0, 1), method='bounded')

        return result.fun
    
    def __find_distances(self, midline, boundary):
        boundary_spline = self.__create_spline(boundary)
        distances_midline = [self.__distance(point, boundary_spline) for point in midline]

        return np.array(distances_midline)
    
    def create_reftrack(self):
        print(self.__points)
        w_l = self.__find_distances(self.__points[self.lines.TRACK], self.__points[self.lines.BLUE]).reshape(-1,1)
        w_r = self.__find_distances(self.__points[self.lines.TRACK], self.__points[self.lines.YELLOW  ]).reshape(-1,1)

        self.__reftrack = np.concatenate((self.__points[self.lines.TRACK], w_r, w_l), axis=-1)

        print(self.__reftrack)

    def is_reftrack_created(self):
        return self.__reftrack != None




# just for testing

def main():

    num_points = 10
    theta = np.linspace(0, 2*np.pi, num_points, endpoint=False)

    l = {}

    for radius in [8,10,9]:
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)
            l[radius] = np.squeeze(np.dstack((x,y)), axis=0)

    
    t = Track()


    t.add_line(points=l[9], line=t.lines.TRACK)
    t.add_line(points=l[10], line=t.lines.BLUE)
    t.add_line(points=l[8], line=t.lines.YELLOW)

    if t.has_boundaries() and not t.is_reftrack_created():
        t.create_reftrack()


if __name__ == "__main__":
    main()