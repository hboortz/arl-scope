import unittest
import rostest
import mock

from quadcopter_brain import QuadcopterBrain, gps_to_mavlink


class TestQuadcopterBrain(unittest.TestCase):
    @unittest.skip('How to mock services?')
    def test_fly_path(self):
        waypoint_data = [
            {'latitude': 1, 'longitude': 2},
            {'latitude': 3, 'longitude': 4},
        ]
        brain = QuadcopterBrain()
        
        brain.fly_path(waypoint_data)

    def test_on_position_update(self):
        pass

class TestMavlinkConversions(unittest.TestCase):
    def test_gps_to_mavlink(self):
        coordinate = 98.7654321
        self.assertEqual(gps_to_mavlink(coordinate), 987654321)

if __name__ == '__main__':
    PKG = 'test_quadcopter_brain'
    rostest.rosrun(PKG, 'test_quadcopter_brain', TestQuadcopterBrain)
    # rostest.rosrun(PKG, 'test_waypoint', TestWaypoint)
    # rostest.rosrun(PKG, 'test_mavlink_conversions', TestMavlinkConversions)