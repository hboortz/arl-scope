import unittest
import rostest
import mock

# import quadcopter_brain
from quadcopter_brain import QuadcopterBrain, build_waypoint, gps_to_mavlink


PKG = 'test_quadcopter_brain'


class TestQuadcopterBrain(unittest.TestCase):
	# @mock.patch('roscopter.msg', mock.MagicMock())
	# @mock.patch('roscopter.srv', mock.MagicMock())
	def test_fly_path(self):
		self.assertTrue(False)

# 	def test_on_position_update(self):
# 		pass

# class TestWaypoint(unittest.TestCase):
# 	def test_build_waypoint(self):
# 		pass

# class TestMavlinkConversions(unittest.TestCase):
# 	def test_gps_to_mavlink(self):
# 		pass

if __name__ == '__main__':
	rostest.rosrun(PKG, 'test_quadcopter_brain', TestQuadcopterBrain)
	# rostest.rosrun(PKG, 'test_waypoint', TestWaypoint)
	# rostest.rosrun(PKG, 'test_mavlink_conversions', TestMavlinkConversions)