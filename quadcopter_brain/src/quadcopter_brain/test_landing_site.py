import datetime
import time
import unittest

from geometry_msgs.msg import Pose, Point
import mock

from quadcopter import Quadcopter
from landing_site import LandingSite
from position_tools import PositionTools


class TestLandingSite(unittest.TestCase):
    @mock.patch('rospy.init_node')
    @mock.patch('rospy.ServiceProxy')
    @mock.patch('rospy.Subscriber')
    def setUp(self, sub_mock, service_mock, init_node_mock):
        self.landing_site = LandingSite()
        self.copter = Quadcopter()
        self.copter.current_lat = 42.0
        self.copter.current_long = -71.0
        self.copter.heading = 0

    # on_fiducial_update and clean_fiducials currently not tested because of
    # the way they interact with ROS components

    def test_find_fiducial_center(self):
        x_coords = [1.5, 4.5, 0.5, -2.5]
        y_coords = [0.75, 3.75, -3.5, 0]
        z_coords = [1, 3, -3, -5]
        pose = self.landing_site.find_fiducial_center(x_coords,
                                                      y_coords,
                                                      z_coords,
                                                      None)
        self.assertAlmostEqual(pose.position.x, 1.0)
        self.assertAlmostEqual(pose.position.y, 0.25)
        self.assertAlmostEqual(pose.position.z, -1.0)

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
    def test_landing_site_get_avg_lat_long(self, lat_long_mock, sleep_mock):
        sleep_mock.side_effect = time.sleep
        self.landing_site.in_view = True
        lat_long_mock.side_effect = [(1, -1), (3, -3), (6, -6)]
        lat, lon = self.landing_site.get_average_lat_long(self.copter,
                                                          total_time=0.3)
        self.assertAlmostEqual(lat, 10.0/3)
        self.assertAlmostEqual(lon, -10.0/3)

        self.landing_site.in_view = False
        lat, lon = self.landing_site.get_average_lat_long(self.copter,
                                                          total_time=0.3)
        self.assertEqual(None, lat)
        self.assertEqual(None, lon)


if __name__ == '__main__':
    unittest.main()
