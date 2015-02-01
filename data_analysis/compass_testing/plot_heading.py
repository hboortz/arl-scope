#!/usr/bin/env pyton

import json
import numpy as np
import matplotlib.pyplot as plt

test_data = ['paul-O-1-25-2015.json',
             'carl-O-1-25-2015.json',
             'paul-SoccerField-1-25-2015.json',
             'carl-SoccerField-1-25-2015.json']
titles = {test_data[0]: 'Paul (Iris+) in Courtyard (the O)',
          test_data[1]: 'Carl (Iris) in Courtyard (the O)',
          test_data[2]: 'Paul (Iris+) on Soccer Field',
          test_data[3]: 'Carl (Iris) on Soccer Field'}


def plot_data(actual, measured, dataset):
    plt.plot([0, 360], [0, 360], 'r', linewidth=2)
    plt.plot(actual, measured, '.', markersize=15)
    plt.title(titles[dataset])
    plt.xlabel('Actual (degrees)')
    plt.ylabel('Iris Compass (degrees)')


def calculate_precision_accuracy(actual, measured, dataset):
    error = np.array([])
    for i in range(len(actual)):
        error = np.append(error, measured[i] - actual[i])
    return dataset, error.mean(), error.std()


def main():
    i = 1
    for test in test_data:
        plt.subplot(2, 2, i)
        i += 1
        fin = open('data/' + test, 'r')
        data = json.load(fin)
        # data saved as nested array: [ [actual], [measured]  ]
        actual = data[0]
        measured = [point / 100.0 for point in data[1]]
        dataset, error_mean, error_std = \
            calculate_precision_accuracy(actual, measured, test)
        print ""
        print "Data from dataset ", dataset
        print "Average error:", error_mean
        print "Std deviation of error:", error_std
        plot_data(actual, measured, test)
    plt.show()


if __name__ == '__main__':
    main()
