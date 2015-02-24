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
        self.assertEqual(gps, -71.1)
        self.assertEqual(type(gps), float)


if __name__ == '__main__':
    unittest.main()
