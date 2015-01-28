import numpy


def center_of_gravity(data):
    average_x = numpy.mean([point[0] for point in data])
    average_y = numpy.mean([point[1] for point in data])
    return numpy.array([average_x, average_y])


def euclidean_distance(p1, p2):
    return numpy.linalg.norm(p1 - p2)


def precision(data):
    cog = center_of_gravity(data)
    distances = numpy.array([euclidean_distance(point, cog) for point in data])
    return numpy.mean(distances)


def simple_moving_average(data, window=5):
    weights = numpy.repeat(1.0, window) / window
    return numpy.convolve(data, weights, 'valid')


def speed_moving_average(times, data):
    print data
    delta_times = times[1:] - times[:-1]
    distances = [euclidean_distance(p1, p2) for p1, p2 in zip(data[1:], data[:-1])]
    print distances
    speeds = [distance / time for distance, time in zip(distances, delta_times)]
    return simple_moving_average(speeds)
