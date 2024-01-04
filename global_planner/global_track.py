import numpy as np
import matplotlib.pyplot as plt
from enum import Enum

FAKE_DISTANCE = 1.5

class Track():
    lines = Enum('Lines', ['TRACK', 'YELLOW', 'BLUE'])
    __points = {}
    __reftrack = None

    def __init__(self, debug = False) -> None:
        self.debug = debug

    def add_line(self, line: lines, points: list):
        self.__points[line] = np.array(points, dtype=np.float)

    def has_boundaries(self) -> bool:
        return self.lines.YELLOW in self.__points and self.lines.BLUE in self.__points
    
    def has_trackline(self) -> bool:
        return self.lines.TRACK in self.__points
    
    def __distance(self, point, segmentsA, segmentsB):
        cross_product = np.cross(segmentsA - point, point - segmentsB)
        norm_b_minus_a = np.linalg.norm(segmentsB - segmentsA, axis=1)
        distance = np.min(np.abs(cross_product) / norm_b_minus_a)
        if self.debug:
            print(f"Min distance for {point}: {distance}")
        return distance if not FAKE_DISTANCE else FAKE_DISTANCE
    
    def __find_distances(self, midline_points, boundaries):
        return np.array([self.__distance(point, boundaries[:-1, :], boundaries[1:, :]) for point in midline_points])
        
    def create_reftrack(self):
        w_l = self.__find_distances(self.__points[self.lines.TRACK], self.__points[self.lines.BLUE]).reshape(-1,1)
        w_r = self.__find_distances(self.__points[self.lines.TRACK], self.__points[self.lines.YELLOW  ]).reshape(-1,1)

        self.__reftrack = np.concatenate((self.__points[self.lines.TRACK], w_r, w_l), axis=-1)

        if self.debug:
            print(self.__reftrack)

    def get_reftrack(self) -> np.ndarray:
        return self.__reftrack

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

    
    t = Track(debug=True)


    t.add_line(points=l[9], line=t.lines.TRACK)
    t.add_line(points=l[10], line=t.lines.BLUE)
    t.add_line(points=l[8], line=t.lines.YELLOW)

    if t.has_boundaries() and not t.is_reftrack_created():
        t.create_reftrack()


if __name__ == "__main__":
    main()