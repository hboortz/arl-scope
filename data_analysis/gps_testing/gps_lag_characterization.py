import os

import numpy

import gps_data

def get_filepath(filename):
    user = os.environ["USER"]
    filepath = (
        "/home/%s/catkin_ws/src/arl-scope/bagfiles/%s" % (user, filename)
    )
    return filepath

def count_timestamps(data):
	timestamp_counts = {}

	for datum in data:
		seconds = int(datum.time)
		timestamp_counts[seconds] = timestamp_counts.get(seconds, 0) + 1

	return timestamp_counts


def main():
	filepath = get_filepath("gps_lag_test-2015-02-08-12-39-28.csv")
	data = gps_data.load_gps_data(filepath)
	counts = count_timestamps(data)
	print("Standard deviation: %f" % (numpy.std(counts.values())))
	print("Maximum: %d" % (max(counts.values())))
	print("Minimum: %d" % (min(counts.values()))) 
	print("Mean: %f" % (numpy.mean(counts.values())))
	print("Median: %d" % (numpy.median(counts.values())))


if __name__ == '__main__':
	main()