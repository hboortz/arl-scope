#!/usr/bin/env pyton

import json
import numpy as np
import matplotlib.pyplot as plt

test_data = ['paul-O-1-25-2015.json',
             'carl-O-1-25-2015.json',
             'paul-SoccerField-1-25-2015.json',
             'carl-SoccerField-1-25-2015.json']

def plot_data(actual, measured, title):
    plt.plot([0, 360], [0, 360], 'r', linewidth=2)
    plt.plot(actual, measured, '.', markersize=15)
    plt.title(title)
    plt.xlabel('Actual (degrees)')
    plt.ylabel('Iris Compass (degrees)')
    plt.show()

def calculate_precision_accuracy(actual, measured, test):
    error = np.array([])
    for i in range(len(actual)):
        error = np.append(error, measured[i] - actual[i])
    print ""
    print "Data for test ", test
    print "Average error:", error.mean()
    print "Std deviation of error:", error.std()

if __name__ == '__main__':
    for test in test_data:
        fin = open('data/' + test, 'r')
        data = json.load(fin)
        # data saved as nested array: [ [actual], [measured]  ]
        actual = data[0]
        measured = [point / 100.0 for point in data[1]]
        calculate_precision_accuracy(actual, measured, test)
        plot_data(actual, measured, test)