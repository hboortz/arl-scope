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
    latlon_diff(42.293004, -71.263612, 42.2929955, -71.2636023)
    latlon_diff(42.293002 , -71.26355, 42.2929737, -71.2636127)

# Put data here as you gather it