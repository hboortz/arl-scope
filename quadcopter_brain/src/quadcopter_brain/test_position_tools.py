#!/usr/bin/env python

import unittest
import math

import mock

from position_tools import PositionTools


class TestPositionTools(unittest.TestCase):
    @mock.patch('geodesy.utm.fromLatLong')
    def test_lat_lon_diff(self, from_lat_long_mock):
        pointA = mock.MagicMock()
        pointA.easting = 3000
        pointA.northing = 4000

        pointB = mock.MagicMock()
        pointB.easting = 3020
        pointB.northing = 3960

        from_lat_long_mock.side_effect = [pointA, pointB]

        dX, dY, distance = PositionTools.lat_lon_diff(1, 2, 3, 4)

        expected = [mock.call(1, 2), mock.call(3, 4)]
        self.assertEqual(from_lat_long_mock.call_args_list, expected)
        self.assertEqual(dX, 20)
        self.assertEqual(dY, -40)
        self.assertAlmostEqual(distance, math.sqrt(2000))

    def test_metered_offset(self):
        # Positions taken from google maps
        new_position =\
            PositionTools.metered_offset(42.293036, -71.263321, 0, -14.78)
        # 5 decimal places is less than a meter
        self.assertAlmostEqual(new_position[0], 42.292903, places=5)
        self.assertAlmostEqual(new_position[1], -71.263319, places=5)
        new_position =\
            PositionTools.metered_offset(42.293009, -71.263531, 157.65, 0)
        # Lower accuracy because of the larger displacement (150m)
        self.assertAlmostEqual(new_position[0], 42.293010, places=4)
        self.assertAlmostEqual(new_position[1], -71.261616, places=4)

    @mock.patch('geodesy.utm.fromLatLong')
    def test_lat_lon_to_meters(self, from_lat_long_mock):
        gps_points = [(0, 0), (1, 1)]
        pointA = mock.MagicMock()
        pointA.easting = 2
        pointA.northing = 3

        pointB = mock.MagicMock()
        pointB.easting = 4
        pointB.northing = 5

        from_lat_long_mock.side_effect = [pointA, pointB]
        metered_positions = PositionTools.lat_lon_to_meters(gps_points)
        self.assertEqual(metered_positions, ([3, 5], [2, 4]))

    def test_gps_to_mavlink(self):
        mavlink_gps = PositionTools.gps_to_mavlink(42.0)
        self.assertEqual(mavlink_gps, 420000000)
        mavlink_gps = PositionTools.gps_to_mavlink(-71.1)
        self.assertEqual(mavlink_gps, -711000000)
        self.assertEqual(type(mavlink_gps), int)

    def test_mavlink_to_gps(self):
        gps = PositionTools.mavlink_to_gps(420000000)
        self.assertEqual(gps, 42.0)
        gps = PositionTools.mavlink_to_gps(-711000000)
        self.assertAlmostEqual(gps, -71.1)

    def test_altitude_to_mavlink(self):
        mavlink_altitude = PositionTools.altitude_to_mavlink(42.3)
        self.assertEqual(mavlink_altitude, 42300)
        self.assertEqual(type(mavlink_altitude), int)

    def test_mavlink_to_altitude(self):
        altitude = PositionTools.mavlink_to_altitude(42300)
        self.assertAlmostEqual(altitude, 42.3)

    def test_degrees_to_mavlink(self):
        mavlink_degrees = PositionTools.degrees_to_mavlink(28.1)
        self.assertEqual(mavlink_degrees, 2810)
        self.assertEqual(type(mavlink_degrees), int)
        with self.assertRaises(ValueError):
            PositionTools.degrees_to_mavlink(-1)

        with self.assertRaises(ValueError):
            PositionTools.degrees_to_mavlink(361)

    def test_mavlink_to_degrees(self):
        degrees = PositionTools.mavlink_to_degrees(2810)
        self.assertAlmostEqual(degrees, 28.1)


if __name__ == '__main__':
    unittest.main()
