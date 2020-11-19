#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

def visualization():
    # load csv file and plot trajectory 
    _, ax = plt.subplots(1)
    ax.set_aspect('equal')

    trajectory = np.loadtxt("trajectory.csv", delimiter=',')
    plt.plot(trajectory[:, 0], trajectory[:, 1], linewidth=2)

    plt.xlim(-1, 5)
    plt.ylim(-1, 5)
    plt.show()

if __name__ == '__main__':
    visualization()
