import csv

import numpy

from filtered_pos import FilteredPos


def load_gps_data(filename):
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        reader.next()
        return [FilteredPos(row) for row in reader]


def extract_gps_coordinates(filtered_poses):
    return numpy.array([(each.latitude,
                         each.longitude) for each in filtered_poses])


def extract_gps_times(filtered_poses):
    times = numpy.array([each.time for each in filtered_poses])
    return times - filtered_poses[0].time
