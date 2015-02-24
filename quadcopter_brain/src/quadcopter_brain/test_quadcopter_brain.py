#!/usr/bin/env python

import unittest

import mock

from quadcopter_brain import QuadcopterBrain


class TestQuadcopterBrain(unittest.TestCase):
    @mock.patch('quadcopter.Quadcopter')
    def setUp(self, quadcopter_mock):
        self.quadcopter_brain = QuadcopterBrain()
        self.quadcopter_mock = quadcopter_mock

    @mock.patch('waypoint_tools.WaypointTools.build_waypoint')
    def test_go_to_waypoints(self, build_waypoint_mock):
        self.assertTrue(True)

    def test_fly_path(self):
        pass

    def test_has_reached_waypoint(self):
        pass

    def test_check_reached_waypoint(self):
        pass

# Todo: what other tests do we need?


if __name__ == '__main__':
    unittest.main()
