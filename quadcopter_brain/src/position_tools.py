#!/usr/bin/env python

import geodesy.utm


class PositionTools():
    @staticmethod
    def lat_lon_diff(latA, lonA, latB, lonB):
        '''
        (latA, lonA): first coordinate
        (latB, lonB): second coordinate
        returns: distance between coordinates in meters as X, Y,
                 and straight line distance
        '''
        pointA = geodesy.utm.fromLatLong(latA, lonA)
        pointB = geodesy.utm.fromLatLong(latB, lonB)
        dX = pointB.easting - pointA.easting
        dY = pointB.northing - pointA.northing
        distance = (dX**2 + dY**2)**0.5
        return dX, dY, distance

    @staticmethod
    def metered_offset(lat, lon, dX, dY):
        '''
        (lat, lon): Starting coordinate
        (dX, dY): offset from starting point in meters
        returns: latitude and longitude of resulting position
        '''
        given_utm = geodesy.utm.fromLatLong(lat, lon)
        given_utm.easting += dX
        given_utm.northing += dY
        return_lat_lon = given_utm.toMsg()
        return return_lat_lon.latitude, return_lat_lon.longitude
