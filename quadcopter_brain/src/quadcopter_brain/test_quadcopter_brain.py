#!/usr/bin/env python

import unittest

import mock

from quadcopter_brain import QuadcopterBrain


class TestQuadcopterBrain(unittest.TestCase):
    @mock.patch('quadcopter.Quadcopter')
    def setUp(self, quadcopter_mock):
        self.quadcopter_brain = QuadcopterBrain()
        self.quadcopter_mock = self.quadcopter_brain.quadcopter

    @mock.patch('rospy.sleep')
    @mock.patch('waypoint_tools.WaypointTools.build_waypoint')
    def test_go_to_waypoints(self, build_waypoint_mock, sleep_mock):
        waypoint_data = [0, 1]
        build_waypoint_mock.side_effect = [10, 11]
        self.quadcopter_brain.go_to_waypoints(waypoint_data)

        expected = [mock.call(0), mock.call(1)]
        self.assertEqual(build_waypoint_mock.call_args_list, expected)

        expected = [mock.call(10), mock.call(11)]
        self.assertEqual(
            self.quadcopter_mock.send_waypoint.call_args_list, expected)

    @mock.patch('quadcopter_brain.QuadcopterBrain.go_to_waypoints')
    def test_fly_path(self, go_to_waypoints_mock):
        waypoint_data = [0, 1]
        self.quadcopter_brain.fly_path(waypoint_data)
        self.quadcopter_mock.launch.assert_called_once_with()
        go_to_waypoints_mock.assert_called_once_with(waypoint_data)
        self.quadcopter_mock.land.assert_called_once_with()

    @mock.patch('quadcopter_brain.QuadcopterBrain.go_to_waypoints')
    def test_go_to_faux_relative_waypoints(self, go_to_waypoint_mock):
        with self.assertRaises(AssertionError) as cm:
            self.quadcopter_brain.go_to_faux_relative_waypoint('a', 1)

        with self.assertRaises(AssertionError) as cm:
            self.quadcopter_brain.go_to_faux_relative_waypoint(1, 'f')

        dEast = 10  # Meters
        dNorth = -10  # Meters
        self.quadcopter_brain.quadcopter.current_lat = 42.0
        self.quadcopter_brain.quadcopter.current_long = -71.0
        self.quadcopter_brain.quadcopter.altitude = 4.5
        self.quadcopter_brain.go_to_faux_relative_waypoint(dEast, dNorth)

        called_waypoint = go_to_waypoint_mock.call_args[0][0][0]
        actual_waypoint = {"latitude": 42.0002612, "longitude": -71.0000939,
                            "altitude": 4.5}  # Taken from google maps

        self.assertAlmostEqual(called_waypoint["latitude"],
                               actual_waypoint["latitude"])
        self.assertAlmostEqual(called_waypoint["longitude"],
                               actual_waypoint["longitude"])
        self.assertAlmostEqual(called_waypoint["altitude"],
                               actual_waypoint["altitude"])

        wait_time = go_to_waypoint_mock.call_args[0][1]
        self.assertAlmostEqual(wait_time, 15)
        

        # expected = [mock.call(0), mock.call(1)]
        # self.assertEqual(build_waypoint_mock.call_args_list, expected)

        # expected = [mock.call(10), mock.call(11)]
        # self.assertEqual(
        #     self.quadcopter_mock.send_waypoint.call_args_list, expected)


if __name__ == '__main__':
    unittest.main()
