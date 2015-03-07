#!/usr/bin/env python

import datetime
import time
import unittest

from geometry_msgs.msg import Pose, Point
import mock

from quadcopter import Quadcopter
from landing_site import LandingSite
from position_tools import PositionTools


class TestLandingSite(unittest.TestCase):
    def setUp(self):
        self.landing_site = LandingSite()
        self.copter = Quadcopter()
        self.copter.current_lat = 42.0
        self.copter.current_long = -71.0
        self.copter.heading = 0

    def test_landing_site_lat_long_same_position(self):
        self.landing_site.center = Pose(position=Point(x=0, y=0, z=6))
        self.copter.heading = 0
        lat, lon = self.landing_site.lat_long(self.copter)
        self.assertAlmostEqual(self.copter.current_lat, lat)
        self.assertAlmostEqual(self.copter.current_long, lon)

    def test_landing_site_lat_long_different_position(self):
        # Format used is [centerX, centerY, heading, dX, dY]
        tests = [[6, -9, 0, 6, 9],
                 [6, -9, 180, -6, -9],
                 [-3, 5, 270, 5, -3]]

        for test in tests:
            self.landing_site.center = Pose(position=Point(x=test[0],
                                                           y=test[1],
                                                           z=6))
            self.copter.heading = test[2]
            lat, lon = self.landing_site.lat_long(self.copter)
            xErr, yErr, dist =\
                PositionTools.lat_long_diff(self.copter.current_lat,
                                           self.copter.current_long,
                                           lat, lon)
            # 1 mm (3 decimals) is a reasonable margin of error
            self.assertAlmostEqual(xErr, test[3], 3)
            self.assertAlmostEqual(yErr, test[4], 3)

    def test_landing_site_lat_long_greater_x(self):
        '''
        latitude shouldn't change (within a few meters)
        longitude should be greater
        '''
        self.landing_site.center = Pose(position=Point(x=1000, y=0, z=6))

        lat, lon = self.landing_site.lat_long(self.copter)
        self.assertAlmostEqual(self.copter.current_lat, lat, 3)
        self.assertGreater(lon, self.copter.current_long)

    def test_landing_site_lat_long_greater_y(self):
        '''
        latitude should be smaller (camera +y is backwards on the copter)
        longitude shouldn't change (within a few meters)
        '''
        self.landing_site.center = Pose(position=Point(x=0, y=1000, z=6))

        lat, lon = self.landing_site.lat_long(self.copter)
        self.assertLess(lat, self.copter.current_lat)
        self.assertAlmostEqual(self.copter.current_long, lon, 3)

    def test_landing_site_lat_long_lesser_x(self):
        '''
        latitude shouldn't change (within a few meters)
        longitude should be bigger
        '''
        self.landing_site.center = Pose(position=Point(x=-1000, y=0, z=6))

        lat, lon = self.landing_site.lat_long(self.copter)
        self.assertAlmostEqual(self.copter.current_lat, lat, 3)
        self.assertLess(lon, self.copter.current_long)

    def test_landing_site_lat_long_lesser_y(self):
        '''
        latitude should be greater (camera -y is forwards on the copter)
        longitude shouldn't change (within a few meters)
        '''
        self.landing_site.center = Pose(position=Point(x=0, y=-1000, z=6))

        lat, lon = self.landing_site.lat_long(self.copter)
        self.assertGreater(lat, self.copter.current_lat)
        self.assertAlmostEqual(lon, self.copter.current_long, 3)

    @mock.patch('rospy.sleep')
    @mock.patch('landing_site.LandingSite.lat_long')
    def test_landing_site_get_avg_lat_long(self, sleep_mock, lat_lon_mock):
        sleep_mock.side_effect = time.sleep(0.1)
        self.landing_site.in_view = True
        # No worky?
        # lat_lon_mock.side_effect = [(1, -1), (3, -3), (6, -6)]
        # Also no worky?
        lat_lon_mock.return_value = (0.0, 0.0)
        lat, lon = self.landing_site.get_average_lat_long(self.copter,
                                                          time=0.2)
        print type(lat), type(lon)
        self.assertAlmostEqual(lat, 10.0/3)
        self.assertAlmostEqual(lon, 10.0/3)




if __name__ == '__main__':
    unittest.main()
