#!/usr/bin/env python

import sys, os
sys.path.append(os.path.relpath('../../quadcopter_brain/src'))
from position_tools import PositionTools

import geodesy.utm
import matplotlib.pyplot as plt


def plot_lat_lon(data, site):
    for spot in data:
        lat,lon = spot
        plt.plot(lat, lon, 'b.', markersize=25)
    plt.title('Error between estimated and tagged landing sites for ' +
               site + ' site')
    plt.xlabel('X (Easting) error between estimation and tag')
    plt.ylabel('Y (Northing) error between estimation and tag')
    plt.show()


def main():
    print "dry spot:"
    plt.figure(1)
    locations = []
    locations.append( PositionTools.lat_lon_diff(42.293004, -71.263612, 42.2929955, -71.2636023) )
    locations.append( PositionTools.lat_lon_diff(42.293004, -71.263612, 42.2930048, -71.2636229) )
    locations.append( PositionTools.lat_lon_diff(42.293004, -71.263612, 42.2929606, -71.2636175) )
    plot_lat_lon(locations, "dry spot")

    print "black line:"
    plt.figure(2)
    locations = []
    locations.append( PositionTools.lat_lon_diff(42.293002 , -71.26355, 42.2929737, -71.2636127) )
    locations.append( PositionTools.lat_lon_diff(42.293002 , -71.26355, 42.2929662, -71.2636271) )
    locations.append( PositionTools.lat_lon_diff(42.293002 , -71.26355, 42.2929698, -71.2636057) )
    plot_lat_lon(locations, "black line")

if __name__ == '__main__':
    main()
# Put data here as you gather it
