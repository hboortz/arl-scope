#!/usr/bin/env pyton

import json
import matplotlib.pyplot as plt


def plot_averages(actual, measured):

    plt.plot(actual,measured)
    plt.show()


if __name__ == '__main__':
    fin = open('data/sfaked_data.json','r')
    data = json.load(fin)
    # data saved as nested array: [ [actual], [measured]  ]
    actual = data[0]
    measured = data[1]
    plot_averages(actual, measured)

