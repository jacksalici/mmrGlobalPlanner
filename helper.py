
import matplotlib.pyplot as plt
import json
import numpy as np
import sys

BAG = False

if BAG:
    points = json.load(open('bag.json'))

    center_line = np.array(points["TRACK"])
    b_boundary = np.array(points["BLUE"])
    y_boundary = np.array(points["YELLOW"])

    plt.plot(center_line[:, 0], center_line[:, 1], 'o-k')
    plt.plot(b_boundary[:, 0], b_boundary[:, 1], 'o-b')
    plt.plot(y_boundary[:, 0], y_boundary[:, 1], 'o-y')

else:
    center_line = np.loadtxt(sys.argv[1], delimiter=',')
    print(center_line)
    plt.plot(center_line[:, 0], center_line[:, 1], 'o-k')







# Show the plot
plt.show()
