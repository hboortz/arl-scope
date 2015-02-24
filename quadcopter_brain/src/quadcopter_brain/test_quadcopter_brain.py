#!/usr/bin/env python

import unittest

import mock

from quadcopter_brain import QuadcopterBrain


class TestQuadcopterBrain(unittest.TestCase):
    @mock.patch('quadcopter.Quadcopter')
    def setUp(self, quadcopter_mock):
        self.quadcopter_brain = QuadcopterBrain()
        self.quadcopter_mock = self.quadcopter_brain.quadcopter

    @mock.patch('waypoint_tools.WaypointTools.build_waypoint')
    def test_go_to_waypoints(self, build_waypoint_mock):
        waypoint_data = [0, 1]
        build_waypoint_mock.side_effect = [10, 11]
        self.quadcopter_brain.go_to_waypoints(waypoint_data)

        expected = [mock.call(0), mock.call(1)]
        self.assertEqual(build_waypoint_mock.call_args_list, expected)

        expected = [mock.call(10), mock.call(11)]
        self.assertEqual(
            self.quadcopter_mock.send_waypoint.call_args_list, expected)

    # def test_fly_path(self):
    #     pass

    # def test_has_reached_waypoint(self):
    #     pass

    # def test_check_reached_waypoint(self):
    #     pass

# Todo: what other tests do we need?


if __name__ == '__main__':
    unittest.main()
