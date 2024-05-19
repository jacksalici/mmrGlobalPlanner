import numpy as np
from enum import Enum


class Track():

    __reftrack = None

    def __init__(self, debug = False) -> None:
        self.debug = debug

    def get_reftrack(self) -> np.ndarray:
        return self.__reftrack
    
    def set_reftrack(self, centerline):
        reftrack = np.array(centerline)
        if (reftrack.shape[1]!=4):
            print("Warning, centerline should be a vector of shape (N,4)")
        self.__reftrack = reftrack

    def is_reftrack_created(self):
        return self.__reftrack != None


# just for testing

def main():
    import matplotlib.pyplot as plt

    num_points = 10
    theta = np.linspace(0, 2*np.pi, num_points, endpoint=False)

    l = {}

    for radius in [8,10,9]:
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        l[radius] = np.squeeze(np.dstack((x,y)), axis=0)

    
    t = Track(debug=True)
    
    distances = [[2] for i in range (len(l[9]))]
    
    t.set_reftrack(np.hstack((l[9], distances, distances)))

    
    points = t.get_reftrack()
    plt.plot(points[:, 0], points[:, 1], 'o-k')
    plt.plot(l[10][:, 0], l[10][:, 1], '^b')
    plt.plot(l[8][:, 0], l[8][:, 1], '^y')
    plt.show()
  

if __name__ == "__main__":
    main()