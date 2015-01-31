import csv

import numpy

from filtered_pos import FilteredPos


def load_gps_data(filename):
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        return [FilteredPos(row) for row in csvreader]

def extract_gps_coordinates(filtered_poses):
    return numpy.array([(each.latitude, 
                         each.longitude) for each in filtered_poses])

def extract_gps_times(filtered_poses):
    return numpy.array([each.time for each in filtered_poses])
