#!/usr/bin/env python

import json
import os
import sys
sys.path.append(os.path.relpath('../../quadcopter_brain/src'))

import matplotlib.pyplot as plt

from position_tools import PositionTools


def plot_lat_lon(latA, lonA, latB, lonB, site):
    '''
    Takes two lists of lat, lon points and plots the difference between
    them, in meters
    '''
    assert (len(latA) == len(latB)) and (len(lonA) == len(lonB))
    error_array = [
        PositionTools.lat_lon_diff(latA[i], lonA[i], latB[i], lonB[i])
        for i in range(len(latA))
    ]
    xError = [data_point[0] for data_point in error_array]
    yError = [data_point[1] for data_point in error_array]

    plt.plot(xError, yError, 'b.', markersize=25)
    plt.title('Error for ' + site)
    plt.xlabel('X (Easting) error between estimation and tag')
    plt.ylabel('Y (Northing) error between estimation and tag')
    plt.show()


def main():
    try:
        with open('1-31-2015-data.json', "r") as f:
            test_runs = json.load(f)
        for test in test_runs:
            plot_lat_lon(test_runs[test]['actual latitude'],
                         test_runs[test]['actual longitude'],
                         test_runs[test]['measured latitude'],
                         test_runs[test]['measured longitude'],
                         test)
    except IOError:
        print "Check that the data file exists"


if __name__ == '__main__':
    main()
