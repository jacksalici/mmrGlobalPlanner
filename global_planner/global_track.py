import numpy as np
import matplotlib.pyplot as plt
from enum import Enum

FAKE_DISTANCE = 1.4

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

    def __track_bound_crossprod(self, waypoint,c_one,c_two):
        ac=waypoint-c_one
        ab=c_two-c_one
        cross_product = np.cross(ab,ac)
        norm_ab = np.linalg.norm(ab)
        norm_cross = np.linalg.norm(cross_product)
        distance = norm_cross / norm_ab
        if self.debug:
            print(f"Min distance for {waypoint}: {distance}")
        return distance 

    def __find_distances(self, center_line, boundary):
        n = len(center_line)
        return np.array([FAKE_DISTANCE for i in range(n)])
    
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
    
    def points_to_file(self, file):
        import json 
        d = {}
        for k, i in self.__points.items():
            d[str(k).split('.')[1]] = i.tolist()
        print(d)
        json.dump(d, open(file, 'w'), 
                indent=4) ### this saves the array in .json format

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
    
    t.points_to_file("test.json")


if __name__ == "__main__":
    main()