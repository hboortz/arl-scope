import unittest
import rostest
import mock

# import quadcopter_brain
from quadcopter_brain import QuadcopterBrain, build_waypoint, gps_to_mavlink


class TestQuadcopterBrain(unittest.TestCase):
    @mock.patch('roscopter.msg')
    @mock.patch('roscopter.srv')
    def test_fly_path(self, srv_mock, msg_mock):
        self.assertTrue(True)

#   def test_on_position_update(self):
#       pass

# class TestWaypoint(unittest.TestCase):
#   def test_build_waypoint(self):
#       pass

class TestMavlinkConversions(unittest.TestCase):
    def test_gps_to_mavlink(self):
        coordinate = 98.7654321
        self.assertEqual(gps_to_mavlink(coordinate), 987654321)

if __name__ == '__main__':
    PKG = 'test_quadcopter_brain'
    rostest.rosrun(PKG, 'test_quadcopter_brain', TestQuadcopterBrain)
    # rostest.rosrun(PKG, 'test_waypoint', TestWaypoint)
    # rostest.rosrun(PKG, 'test_mavlink_conversions', TestMavlinkConversions)