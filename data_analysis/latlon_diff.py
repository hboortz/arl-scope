#!/usr/bin/env python

import geodesy.utm

def latlon_diff(latA, lonA, latB, lonB):
    pointA = geodesy.utm.fromLatLong(latA, lonA)
    pointB = geodesy.utm.fromLatLong(latB, lonB)

    dX = pointA.easting - pointB.easting
    dY = pointA.northing - pointB.northing

    print "Differences (m): %f, %f" % (dX, dY)
    print "Distance (m): ", (dX**2 + dY**2)**0.5

if __name__ == '__main__':
    latlon_diff(42., -71., 42.3333, -71.7777)

# Put data here as you gather it