#import geodesy.utm


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
        return int(altitude * 1e3)

    @staticmethod
    def mavlink_to_altitude(int_altitude):
        '''
        coordinate: integer altitude from mavlink (mm)
        returns altitude in meters
        '''
        return int_altitude / 1e3

    @staticmethod
    def degrees_to_mavlink(heading):
        '''
        heading: decimal degrees, 0-360
        '''
        if heading < 0 or heading > 360:
            raise ValueError

        return int(heading * 1e2)

    @staticmethod
    def mavlink_to_degrees(int_heading):
        '''
        int_heading: integer heading from mavlink
        returns decimal degrees, 0-360
        '''
        return int_heading / 1e2


class QuadcopterPose(object):
    def __init__(self, latitude, longitude, altitude, relative_altitude, heading):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.relative_altitude = relative_altitude
        self.heading = heading

    @classmethod
    def from_mavlink(cls, msg):
        return QuadcopterPose(
            PositionTools.mavlink_to_gps(msg.latitude),
            PositionTools.mavlink_to_gps(msg.longitude),
            PositionTools.mavlink_to_altitude(msg.altitude),
            PositionTools.mavlink_to_altitude(msg.relative_altitude),
            PositionTools.mavlink_to_degrees(msg.heading))

    def __add__(self, rhs):
        assert(isinstance(rhs, QuadcopterPoseOffset))
        return QuadcopterPose(
            self.latitude + rhs.latitude,
            self.longitude + rhs.longitude,
            self.altitude + rhs.altitude,
            self.relative_altitude + rhs.relative_altitude,
            self.heading + rhs.heading)
    def __sub__(self, rhs):
        assert(isinstance(rhs, QuadcopterPose))
        return QuadcopterPoseOffset(
            self.latitude - rhs.latitude,
            self.longitude - rhs.longitude,
            self.altitude - rhs.altitude,
            self.relative_altitude - rhs.relative_altitude,
            self.heading - rhs.heading)

    def __str__(self):
        return "({0}, {1}) z={2} theta={3}".format(self.latitude, self.longitude, self.relative_altitude, self.heading)
    __repr__ = __str__


class QuadcopterPoseOffset(QuadcopterPose):
    def __add__(self, rhs):
        if isinstance(rhs, QuadcopterPoseOffset):
            return QuadcopterPoseOffset(
                self.latitude + rhs.latitude,
                self.longitude + rhs.longitude,
                self.altitude + rhs.altitude,
                self.relative_altitude + rhs.relative_altitude,
                self.heading + rhs.heading)
        return QuadcopterPose(
            self.latitude + rhs.latitude,
            self.longitude + rhs.longitude,
            self.altitude + rhs.altitude,
            self.relative_altitude + rhs.relative_altitude,
            self.heading + rhs.heading)
    def __mul__(self, rhs):
        return QuadcopterPoseOffset(
            self.latitude * rhs,
            self.longitude * rhs,
            self.altitude * rhs,
            self.relative_altitude * rhs,
            self.heading * rhs)
