#!/usr/bin/env python

import unittest

import mock
import roscopter.msg

from waypoint_tools import WaypointTools


class TestWaypointTools(unittest.TestCase):
    def test_build_waypoint(self):
        waypoint_data = {"latitude": 42.12345,
                         "longitude": -71.98765}
        waypoint = WaypointTools.build_waypoint(waypoint_data)
        self.assertEqual(type(waypoint), roscopter.msg.Waypoint)
        self.assertAlmostEqual(waypoint.latitude, 421234500)
        self.assertAlmostEqual(waypoint.longitude, -719876500)
        self.assertEqual(waypoint.altitude, 8000)
        self.assertEqual(waypoint.hold_time, 3000)

        waypoint_data = {"latitude": 42.12345,
                         "longitude": -71.98765,
                         "altitude": 1.0, 
                         "hold_time": 10.0}
        waypoint = WaypointTools.build_waypoint(waypoint_data)
        self.assertEqual(type(waypoint), roscopter.msg.Waypoint)
        self.assertEqual(waypoint.latitude, 421234500)
        self.assertEqual(waypoint.longitude, -719876500)
        self.assertEqual(waypoint.altitude, 1000)
        self.assertEqual(waypoint.hold_time, 10000)

    def test_open_waypoint_file(self):
        A = {"A": {"latitude": 42.292695, "longitude": -71.263091}}
        waypoints_from_file =\
            WaypointTools.open_waypoint_file("great_lawn_waypoints.json")
        self.assertEqual(A["A"], waypoints_from_file["A"])


if __name__ == '__main__':
    unittest.main()
