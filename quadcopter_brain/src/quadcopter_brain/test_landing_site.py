#!/usr/bin/env python

import unittest

import mock
from geometry_msgs.msg import Pose, Point

from quadcopter import Quadcopter
from landing_site import LandingSite
from position_tools import PositionTools


class TestLandingSite(unittest.TestCase):
    @mock.patch('rospy.init_node')
    def setUp(self, init_node_mock):
        self.landing_site = LandingSite()
        self.quadcopter = Quadcopter()
        self.quadcopter.current_lat = 42.0
        self.quadcopter.current_long = -71.0
        self.quadcopter.heading = 0

    def test_landing_site_lat_lon_same_position(self):
        self.landing_site.center = Pose(position=Point(x=0, y=0, z=6))
        self.quadcopter.heading = 0
        lat, lon = self.landing_site.lat_long(self.quadcopter)
        self.assertAlmostEqual(self.quadcopter.current_lat, lat)
        self.assertAlmostEqual(self.quadcopter.current_long, lon)

    def test_landing_site_lat_long_different_position(self):
        # Format used is [centerX, centerY, heading, dX, dY]
        tests = [[6, -9, 0, 6, 9],
                 [6, -9, 180, -6, -9],
                 [-3, 5, 270, 5, -3]]

        for test in tests:
            self.landing_site.center = Pose(position=Point(x=test[0],
                                                           y=test[1],
                                                           z=6))
            self.quadcopter.heading = test[2]
            lat, lon = self.landing_site.lat_long(self.quadcopter)
            xErr, yErr, dist =\
                PositionTools.lat_long_diff(self.quadcopter.current_lat,
                                           self.quadcopter.current_long,
                                           lat, lon)
            # 1 mm (3 decimals) is a reasonable margin of error
            self.assertAlmostEqual(xErr, test[3], 3)
            self.assertAlmostEqual(yErr, test[4], 3)

    def test_landing_site_lat_lon_greater_x(self):
        '''
        latitude shouldn't change (within a few meters)
        longitude should be greater
        '''
        self.landing_site.center = Pose(position=Point(x=1000, y=0, z=6))

        lat, lon = self.landing_site.lat_long(self.quadcopter)
        self.assertAlmostEqual(self.quadcopter.current_lat, lat, 3)
        self.assertGreater(lon, self.quadcopter.current_long)

    def test_landing_site_lat_lon_greater_y(self):
        '''
        latitude should be smaller (camera +y is backwards on the copter)
        longitude shouldn't change (within a few meters)
        '''
        self.landing_site.center = Pose(position=Point(x=0, y=1000, z=6))

        lat, lon = self.landing_site.lat_long(self.quadcopter)
        self.assertLess(lat, self.quadcopter.current_lat)
        self.assertAlmostEqual(self.quadcopter.current_long, lon, 3)

    def test_landing_site_lat_lon_lesser_x(self):
        '''
        latitude shouldn't change (within a few meters)
        longitude should be bigger
        '''
        self.landing_site.center = Pose(position=Point(x=-1000, y=0, z=6))

        lat, lon = self.landing_site.lat_long(self.quadcopter)
        self.assertAlmostEqual(self.quadcopter.current_lat, lat, 3)
        self.assertLess(lon, self.quadcopter.current_long)

    def test_landing_site_lat_lon_lesser_y(self):
        '''
        latitude should be greater (camera -y is forwards on the copter)
        longitude shouldn't change (within a few meters)
        '''
        self.landing_site.center = Pose(position=Point(x=0, y=-1000, z=6))

        lat, lon = self.landing_site.lat_long(self.quadcopter)
        self.assertGreater(lat, self.quadcopter.current_lat)
        self.assertAlmostEqual(lon, self.quadcopter.current_long, 3)


if __name__ == '__main__':
    unittest.main()
