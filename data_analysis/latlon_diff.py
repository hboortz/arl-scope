#!/usr/bin/env python

import geodesy.utm
import matplotlib.pyplot as plt


def latlon_diff(latA, lonA, latB, lonB):
    pointA = geodesy.utm.fromLatLong(latA, lonA)
    pointB = geodesy.utm.fromLatLong(latB, lonB)

    dX = pointA.easting - pointB.easting
    dY = pointA.northing - pointB.northing

    print "Differences (m): %f, %f" % (dX, dY)
    print "Distance (m): ", (dX**2 + dY**2)**0.5
    return dX,dY

def plot_latlon(data, site):
    for spot in data:
        lat,lon = spot
        plt.plot(lat, lon, 'b.', markersize=25)
    plt.title('Error between estimated and tagged landing sites for ' +
               site + ' site')
    plt.xlabel('X (Easting) error between estimation and tag')
    plt.ylabel('Y (Northing) error between estimation and tag')
    plt.show()

if __name__ == '__main__':
    print "dry spot:"
    plt.figure(1)
    locations = []
    locations.append( latlon_diff(42.293004, -71.263612, 42.2929955, -71.2636023) )
    locations.append( latlon_diff(42.293004, -71.263612, 42.2930048, -71.2636229) )
    locations.append( latlon_diff(42.293004, -71.263612, 42.2929606, -71.2636175) )
    plot_latlon(locations, "dry spot")

    print "black line:"
    plt.figure(2)
    locations = []
    locations.append( latlon_diff(42.293002 , -71.26355, 42.2929737, -71.2636127) )
    locations.append( latlon_diff(42.293002 , -71.26355, 42.2929662, -71.2636271) )
    locations.append( latlon_diff(42.293002 , -71.26355, 42.2929698, -71.2636057) )
    plot_latlon(locations, "black line")

# Put data here as you gather it