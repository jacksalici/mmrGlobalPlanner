import numpy as np
import matplotlib.pyplot as plt
from enum import Enum


class Track():
    lines = Enum('Lines', ['YELLOW', 'BLUE'])
    __points = {}
    __reftrack = None

    def __init__(self, debug = False) -> None:
        self.debug = debug

    def add_line(self, line: lines, points: list):
        self.__points[line] = np.array(points, dtype=np.float)

    def has_boundaries(self) -> bool:
        return self.lines.YELLOW in self.__points and self.lines.BLUE in self.__points

    def get_reftrack(self) -> np.ndarray:
        return self.__reftrack
    
    def set_reftrack(self, centerline):
        reftrack = np.array(centerline)
        if (reftrack.shape[1]!=4):
            print("Warning, centerline should be a vector of shape (N,4)")
        self.__reftrack = reftrack

    def is_reftrack_created(self):
        return self.__reftrack != None
    
    def points_to_file(self, file):
        import json 
        d = {}
        for k, i in self.__points.items():
            d[str(k).split('.')[1]] = i.tolist()
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
    
    t.set_reftrack(l[9])
    t.add_line(points=l[10], line=t.lines.BLUE)
    t.add_line(points=l[8], line=t.lines.YELLOW)

    
    points = t.get_reftrack()
    plt.plot(points[:, 0], points[:, 1], 'o-k')
    plt.plot(l[10][:, 0], l[10][:, 1], '^b')
    plt.plot(l[8][:, 0], l[8][:, 1], '^y')
    plt.show()
  

if __name__ == "__main__":
    main()