import geodesy.utm


class PositionTools():
    @staticmethod
    def lat_lon_diff(latA, lonA, latB, lonB):
        '''
        (latA, lonA): first coordinate
        (latB, lonB): second coordinate
        returns: distance between coordinates in meters as X, Y, and straight
                 line distance
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

    @staticmethod
    def lat_lon_to_meters(gps_points):
        '''
        Takes a list of (lat, lon) GPS points and returns a list of (x, y)
        points in meters (UTM)
        '''
        utm_points = \
            [geodesy.utm.fromLatLong(lat, lon) for lat, lon in gps_points]
        x_metered_points = [point.northing for point in utm_points]
        y_metered_points = [point.easting for point in utm_points]
        return (x_metered_points, y_metered_points)

    @staticmethod
    def gps_to_mavlink(coordinate):
        '''
        coordinate: decimal degrees
        returns an integer representation of lat/long that mavlink wants
        '''
        return int(coordinate * 1e7)

    @staticmethod
    def mavlink_to_gps(int_coordinate):
        '''
        coordinate: integer lat/long from mavlink
        returns decimal degrees
        '''
        return int_coordinate / 1e7

    @staticmethod
    def altitude_to_mavlink(altitude):
        '''
        altitude: altitude in meters
        returns integer altitude from mavlink (mm)
        '''
        return int(coordinate * 1e3)

    @staticmethod
    def mavlink_to_altitude(int_altitude):
        '''
        coordinate: integer altitude from mavlink (mm)
        returns altitude in meters
        '''
        return int_coordinate / 1e3

    @staticmethod
    def degrees_to_mavlink(heading):
        '''
        heading: decimal degrees, 0-360
        '''
        return int(coordinate * 1e2)

    @staticmethod
    def mavlink_to_degrees(int_heading):
        '''
        int_heading: integer heading from mavlink
        returns decimal degrees, 0-360
        '''
        return int_coordinate / 1e2
