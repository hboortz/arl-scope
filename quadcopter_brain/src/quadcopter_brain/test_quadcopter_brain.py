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
    def test_go_to_waypoint_given_metered_offset(self, go_to_waypoint_mock):
        dEast = 10  # Meters
        dNorth = -10  # Meters
        self.quadcopter_brain.quadcopter.current_lat = 42.0
        self.quadcopter_brain.quadcopter.current_long = -71.0
        self.quadcopter_brain.quadcopter.current_rel_alt = 4.5
        self.quadcopter_brain.go_to_waypoint_given_metered_offset(dEast,
                                                                  dNorth)

        called_waypoint = go_to_waypoint_mock.call_args[0][0][0]
        actual_waypoint = {"latitude": 41.999912, "longitude": -70.999877,
                           "altitude": 4.5}  # Taken from google maps

        self.assertAlmostEqual(called_waypoint["latitude"],
                               actual_waypoint["latitude"], 6)
        self.assertAlmostEqual(called_waypoint["longitude"],
                               actual_waypoint["longitude"], 6)
        self.assertAlmostEqual(called_waypoint["altitude"],
                               actual_waypoint["altitude"])

        wait_time = go_to_waypoint_mock.call_args[0][1]
        self.assertAlmostEqual(wait_time, 15)

        dEast = -10  # Meters
        dNorth = 10  # Meters
        dAlt = 2  # Meters
        sleep_time = 10  # Seconds
        self.quadcopter_brain.go_to_waypoint_given_metered_offset(dEast,
                                                                  dNorth,
                                                                  dAlt,
                                                                  sleep_time)

        called_waypoint = go_to_waypoint_mock.call_args[0][0][0]
        actual_waypoint = {"latitude": 42, "longitude": -71,
                           "altitude": 6.5}  # Taken from google maps

        self.assertNotEqual(called_waypoint["latitude"],
                            actual_waypoint["latitude"], 6)
        self.assertNotEqual(called_waypoint["longitude"],
                            actual_waypoint["longitude"], 6)
        self.assertAlmostEqual(called_waypoint["altitude"],
                               actual_waypoint["altitude"])

        wait_time = go_to_waypoint_mock.call_args[0][1]
        self.assertAlmostEqual(wait_time, 10)


    @mock.patch('quadcopter_brain.QuadcopterBrain.go_to_waypoints')
    @mock.patch('quadcopter_brain.QuadcopterBrain.find_landing_site')
    def test_land_on_fiducial_incremental(self, find_mock, go_to_mock):
        pass
        # Tests not found case
        # find_mock.return_value = False, 0, 0
        # self.quadcopter_brain.land_on_fiducial_incremental()
        # assert not self.landing_site_mock.get_average_lat_long.called
        # self.quadcopter_mock.land.assert_called_once_with()

        # Tests found, not seen AND too high case

        # Tests found, not seen AND low enough case

        # Tests a successful run

        # Tests an unsuccessful run


if __name__ == '__main__':
    unittest.main()
