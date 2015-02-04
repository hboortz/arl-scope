import numpy
import geodesy.utm


def center_of_gravity(data):
    average_x = numpy.mean([point[0] for point in data])
    average_y = numpy.mean([point[1] for point in data])
    return numpy.array([average_x, average_y])


def latlon_diff(latA, lonA, latB, lonB):
    pointA = geodesy.utm.fromLatLong(latA, lonA)
    pointB = geodesy.utm.fromLatLong(latB, lonB)

    dX = pointB.easting - pointA.easting
    dY = pointB.northing - pointA.northing

    return (dX**2 + dY**2)**0.5


def euclidean_distance(p1, p2):
    return latlon_diff(p1[0], p1[1], p2[0], p2[1])


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
