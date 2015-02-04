#!/usr/bin/env python

import unittest

from geometry_msgs.msg import Pose, Point
from quadcopter_fiducial_brain import QuadcopterFiducialBrain
from landing_site import LandingSite

from position_tools import PositionTools


class TestLandingSite(unittest.TestCase):
    def setUp(self):
        self.landing_site = LandingSite()
        self.brain = QuadcopterFiducialBrain()
        self.brain.latitude = 42.0
        self.brain.longitude = -71.0

    def test_landing_site_lat_lon_same_position(self):
        self.landing_site.center = Pose(position=Point(x=0, y=0, z=6))
        self.brain.heading = 0
        lat, lon = self.landing_site.lat_lon(self.brain)
        self.assertAlmostEqual(self.brain.latitude, lat)
        self.assertAlmostEqual(self.brain.longitude, lon)

    def test_landing_site_lat_lon_different_position(self):
        # Format used is [centerX, centerY, heading, dX, dY]
        tests = [[6, -9, 0, 6, 9],
                 [6, -9, 180, -6, -9],
                 [-3, 5, 270, 5, -3]]
        for test in tests:
            self.landing_site.center = Pose(position=Point(x=test[0],
                                                           y=test[1],
                                                           z=6))
            self.brain.heading = test[2]
            lat, lon = self.landing_site.lat_lon(self.brain)
            xErr, yErr, dist =\
                PositionTools.lat_lon_diff(self.brain.latitude,
                                           self.brain.longitude,
                                           lat, lon)
            self.assertAlmostEqual(xErr, test[3], 3)  # 3 is acceptable (mm)
            self.assertAlmostEqual(yErr, test[4], 3)


if __name__ == '__main__':
    unittest.main()
