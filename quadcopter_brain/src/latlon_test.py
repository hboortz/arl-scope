#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point
from quadcopter_fiducial_brain import QuadcopterFiducialBrain
from landing_site import LandingSite
import geodesy.utm

def latlon_diff(latA, lonA, latB, lonB):
    pointA = geodesy.utm.fromLatLong(latA, lonA)
    pointB = geodesy.utm.fromLatLong(latB, lonB)

    dX = pointB.easting - pointA.easting
    dY = pointB.northing - pointA.northing

    print "Differences (m): %f, %f" % (dX, dY)
    print "Distance (m): ", (dX**2 + dY**2)**0.5

def main():
    derpy = QuadcopterFiducialBrain()
    moon = LandingSite()

    derpy.latitude = 42.0
    derpy.longitude = -71.0
    derpy.heading = 315
    
    moon.center = Pose(position=Point(x=6, y=6, z=6))

    lat,lon = moon.latlon(derpy)
    latlon_diff(derpy.latitude, derpy.longitude, lat, lon)

if __name__ == '__main__':
    main()