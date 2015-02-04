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

    def test_landing_site_lat_lon_same_position(self):
        self.landing_site.center = Pose(position=Point(x=0, y=0, z=6))

        self.brain.latitude = 42.0
        self.brain.longitude = -71.0
        self.brain.heading = 0

        lat, lon = self.landing_site.lat_lon(self.brain)
        self.assertAlmostEqual(self.brain.latitude, lat)
        self.assertAlmostEqual(self.brain.longitude, lon)

    def test_landing_site_lat_lon_greater_x(self):
        '''
        latitude shouldn't change (within 1m)   
        longitude should be greater [TODO: why is it smaller?]
        '''
        self.landing_site.center = Pose(position=Point(x=10, y=0, z=6))

        self.brain.latitude = 42.0
        self.brain.longitude = -71.0
        self.brain.heading = 0

        lat, lon = self.landing_site.lat_lon(self.brain)
        self.assertAlmostEqual(self.brain.latitude, lat, 5)
        self.assertLess(self.brain.longitude, lon)

    def test_landing_site_lat_lon_greater_y(self):
        '''
        latitude should be greater
        longitude shouldn't change (within 1m)
        '''
        self.landing_site.center = Pose(position=Point(x=0, y=10, z=6))

        self.brain.latitude = 42.0
        self.brain.longitude = -71.0
        self.brain.heading = 0

        lat, lon = self.landing_site.lat_lon(self.brain)
        self.assertGreater(self.brain.latitude, lat)
        self.assertAlmostEqual(self.brain.longitude, lon, 5)

    def test_landing_site_lat_lon_lesser_x(self):
        '''
        latitude shouldn't change (within 1m)   
        longitude should be smaller [TODO: why is it bigger?]
        '''
        self.landing_site.center = Pose(position=Point(x=-10, y=0, z=6))

        self.brain.latitude = 42.0
        self.brain.longitude = -71.0
        self.brain.heading = 0

        lat, lon = self.landing_site.lat_lon(self.brain)
        self.assertAlmostEqual(self.brain.latitude, lat, 5)
        self.assertGreater(self.brain.longitude, lon)

    def test_landing_site_lat_lon_lesser_y(self):
        '''
        latitude should be smaller
        longitude shouldn't change (within 1m)
        '''
        self.landing_site.center = Pose(position=Point(x=0, y=-10, z=6))

        self.brain.latitude = 42.0
        self.brain.longitude = -71.0
        self.brain.heading = 0

        lat, lon = self.landing_site.lat_lon(self.brain)
        self.assertLess(self.brain.latitude, lat)
        self.assertAlmostEqual(self.brain.longitude, lon, 5)

    def test_landing_site_heading(self):
        '''
        latitude should
        longitude should
        [TODO: pick heading and resulting lat/lon]
        '''
        self.landing_site.center = Pose(position=Point(x=0, y=-10, z=6))

        self.brain.latitude = 42.0
        self.brain.longitude = -71.0
        self.brain.heading = 0

        lat, lon = self.landing_site.lat_lon(self.brain)
        self.assertLess(self.brain.latitude, lat)
        self.assertAlmostEqual(self.brain.longitude, lon, 5)

    




if __name__ == '__main__':
    unittest.main()
