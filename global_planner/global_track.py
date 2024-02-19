import numpy as np
import matplotlib.pyplot as plt
from enum import Enum


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

    def __find_distances(self, center_line, boundary):
        FAKE_DISTANCE = 1.4
        n = len(center_line)
        return np.array([FAKE_DISTANCE for i in range(n)])
    
    def create_reftrack(self):
        w_l = self.__find_distances(self.__points[self.lines.TRACK], self.__points[self.lines.BLUE]).reshape(-1,1)
        w_r = self.__find_distances(self.__points[self.lines.TRACK], self.__points[self.lines.YELLOW  ]).reshape(-1,1)

        self.__reftrack = np.concatenate((self.__points[self.lines.TRACK], w_r, w_l), axis=-1)

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
    
    points = t.get_reftrack()

    plt.plot(points[:, 0], points[:, 1], 'o-k')
    plt.show()
  

if __name__ == "__main__":
    main()