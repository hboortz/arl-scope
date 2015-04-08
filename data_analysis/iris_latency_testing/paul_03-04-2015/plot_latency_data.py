#!/usr/bin/env pyton

import json
import numpy as np
import matplotlib.pyplot as plt

test_data = ['1-HighBandwidthCompassResults.json',
             '2-HighBandwidthAltitudeResults.json',
             '3-LowLatencyCompassResults.json']
ylabels = {test_data[0]: 'Reported Compass (Degrees)',
           test_data[1]: 'Reported Relative altitude (mm)',
           test_data[2]: 'Reported Compass (Degrees)'}
titles = {test_data[0]: 'High Bandwidth Compass Lag (Paul in the O)',
          test_data[1]: 'High Bandwidth Altitude Lag (Paul in the O)',
          test_data[2]: 'Low-Latency Compass Lag (Paul in the O)'}
colors = ['b', 'g', 'k', 'r', 'c']


def plot_data(data, index, dataset, compass):
    time, actual, measured = process_data(data, index + 1, compass)
    plt.subplot(2, 1, 1)
    plt.title(titles[dataset])
    plt.plot(time, actual, '.', markersize=15, color=colors[index])
    plt.plot(time, actual, color=colors[index])
    plt.xlabel('Time (seconds)')
    plt.ylabel('Actual Motion (0: start, 1: end of travel)')
    plt.axis([0, 3, -0.1, 1.1])
    plt.subplot(2, 1, 2)
    plt.plot(time, measured, '.', markersize=15, color=colors[index])
    plt.plot(time, measured, color=colors[index])
    plt.xlabel('Time (seconds)')
    plt.ylabel(ylabels[dataset])


def process_data(data, index, compass):
    time = data[str(index) + '_time']
    offset = time[0]
    time = [moment - offset for moment in time]
    actual = data[str(index) + '_actual']
    if compass:
        measured = process_compass(data[str(index) + '_measured'])
    else:
        measured = process_altitude(data[str(index) + '_measured'])
    return time, actual, measured


def process_compass(compass_data):
    compass_data = [datum / 100.0 for datum in compass_data]
    compass_data = [(datum if datum < 275 else datum - 360) for datum in compass_data]
    return compass_data


def process_altitude(altitude_data):
    offset = altitude_data[0]
    return [point - offset for point in altitude_data]


def main():
    for test in test_data:
        plt.figure()
        fin = open(test, 'r')
        data = json.load(fin)
        for i in range(5):
            if test.count('Compass') > 0:
                plot_data(data, i, test, True)
            else:
                plot_data(data, i, test, False)
        plt.show()


if __name__ == '__main__':
    main()
