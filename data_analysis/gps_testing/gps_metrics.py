import numpy
import geodesy.utm

from quadcopter_brain import position_tools


def center_of_gravity(data):
    average_x = numpy.mean([point[0] for point in data])
    average_y = numpy.mean([point[1] for point in data])
    return numpy.array([average_x, average_y])


def euclidean_distance(p1, p2):
    '''
    Takes in two lat/lon pairs and returns distance in meters between them
    '''
    _, _, dist = position_tools.PositionTools.lat_lon_diff(
        p1[0], p1[1], p2[0], p2[1]
    )
    return dist


def precision(data):
    cog = center_of_gravity(data)
    distances = numpy.array([euclidean_distance(point, cog) for point in data])
    return numpy.mean(distances)


def speeds(times, data):
    delta_times = times[1:] - times[:-1]
    distances = [euclidean_distance(p1, p2) for p1, p2 in zip(data[1:],
                                                              data[:-1])]
    return [distance / time for distance, time in zip(distances, delta_times)]


def average_speed(times, data):
    return numpy.mean(speeds(times, data))
