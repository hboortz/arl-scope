#!/usr/bin/env python

import unittest

import mock

from quadcopter_brain import QuadcopterBrain
import rc_command


class TestQuadcopterBrain(unittest.TestCase):
    @mock.patch('landing_site.LandingSite')
    @mock.patch('quadcopter.Quadcopter')
    def setUp(self, quadcopter_mock, landing_site_mock):
        self.quadcopter_brain = QuadcopterBrain()
        self.quadcopter_mock = self.quadcopter_brain.quadcopter
        self.landing_site_mock = self.quadcopter_brain.landing_site

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
        delta_east = 10  # Meters
        delta_north = -10  # Meters
        self.quadcopter_brain.quadcopter.current_lat = 42.0
        self.quadcopter_brain.quadcopter.current_long = -71.0
        self.quadcopter_brain.quadcopter.current_rel_alt = 4.5
        self.quadcopter_brain.go_to_waypoint_given_metered_offset(delta_east,
                                                                  delta_north)
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

        delta_east = -10  # Meters
        delta_north = 10  # Meters
        delta_alt = 2  # Meters
        sleep_time = 10  # Seconds
        self.quadcopter_brain.go_to_waypoint_given_metered_offset(delta_east,
                                                                  delta_north,
                                                                  delta_alt,
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

    @mock.patch('rospy.sleep')
    def test_find_landing_site(self, sleep_mock):
        # Test what happens when seen
        self.landing_site_mock.in_view = True
        self.landing_site_mock.lat_long.return_value = (-42, 71)
        res = self.quadcopter_brain.find_landing_site()
        self.assertEqual(res, (True, -42, 71))

        # Test what happens when not seen
        self.landing_site_mock.in_view = False
        self.landing_site_mock.lat_long.return_value = (-42, 71)
        res = self.quadcopter_brain.find_landing_site(1)
        self.assertEqual(res, (False, 0, 0))

        # Test what happens when seen after a few tries
        in_view_mock = mock.PropertyMock(side_effect=[False, False, True])
        type(self.landing_site_mock).in_view = in_view_mock
        self.landing_site_mock.lat_long.return_value = (-42, 71)
        sleep_mock.call_args_list = []
        res = self.quadcopter_brain.find_landing_site()
        expected_sleep_calls = [mock.call(0.1), mock.call(0.1), mock.call(0.1)]
        self.assertEqual(sleep_mock.call_args_list, expected_sleep_calls)
        self.assertEqual(res, (True, -42, 71))

    @mock.patch('quadcopter_brain.QuadcopterBrain.go_to_waypoints')
    @mock.patch('quadcopter_brain.QuadcopterBrain.find_landing_site')
    def test_land_on_fiducial_simple(self, find_mock, go_to_mock):
        # Fiducial found during landing
        find_mock.return_value = True, 42, 71
        self.quadcopter_brain.land_on_fiducial_simple()
        wpt = {'latitude': 42,
               'longitude': 71,
               'altitude': 1.0}
        go_to_mock.assert_called_once_with([wpt])
        self.quadcopter_mock.land.assert_called_once_with()

        # Fiducial not found during landing
        go_to_mock.reset_mock()
        self.quadcopter_mock.land.reset_mock()
        find_mock.return_value = False, 0, 0
        self.quadcopter_brain.land_on_fiducial_simple()
        assert not go_to_mock.called
        self.quadcopter_mock.land.assert_called_once_with()

    @mock.patch('quadcopter_brain.QuadcopterBrain.find_landing_site')
    @mock.patch('quadcopter_brain.QuadcopterBrain.go_to_waypoints')
    def test_find_landing_site_at_waypoints(self, go_to_mock, find_site_mock):
        waypoint_data = [0, 1]
        find_site_mock.return_value = False, 0, 0
        res = \
            self.quadcopter_brain.find_landing_site_at_waypoints(waypoint_data)
        go_to_expected = [mock.call([pt]) for pt in waypoint_data]
        self.assertEqual(go_to_mock.call_args_list, go_to_expected)
        find_site_expected = [mock.call(15) for point in waypoint_data]
        self.assertEqual(find_site_mock.call_args_list, find_site_expected)
        self.assertEqual(res, (False, 0, 0))

        go_to_mock.reset_mock()
        find_site_mock.reset_mock()

        find_site_mock.return_value = True, 42.0, -71.0
        res = \
            self.quadcopter_brain.find_landing_site_at_waypoints(waypoint_data)
        go_to_mock.assert_called_once_with([0])
        find_site_mock.assert_called_once_with(15)
        self.assertEqual(res, (True, 42.0, -71.0))

    def test_send_rc_command(self):
        self.quadcopter_brain.send_rc_command(0.5, 0.5, 0.5)
        self.assertEqual(
            len(self.quadcopter_mock.send_rc_command.call_args_list), 1)

    def test_get_planar_speed(self):
        self.assertAlmostEqual(self.quadcopter_brain.get_planar_speed(0), 0.5)
        self.assertAlmostEqual(
            self.quadcopter_brain.get_planar_speed(10), 0.9, delta=0.01)
        self.assertAlmostEqual(
            self.quadcopter_brain.get_planar_speed(-10), 0.1, delta=0.01)

    def test_get_rate_of_descent(self):
        self.assertAlmostEqual(
            self.quadcopter_brain.get_rate_of_descent(10, 10), 0.5, delta=0.01)
        self.assertAlmostEqual(
            self.quadcopter_brain.get_rate_of_descent(0, 0), 0.25, delta=0.01)

    @mock.patch('quadcopter_brain.QuadcopterBrain.get_planar_speed')
    @mock.patch('quadcopter_brain.QuadcopterBrain.get_rate_of_descent')
    @mock.patch('quadcopter_brain.QuadcopterBrain.send_rc_command')
    def test_proportional_position(self, command_mock, descent_mock,
                                   planar_mock):
        planar_mock.side_effect = [0.1, 0.9]
        descent_mock.return_value = 0.3
        self.quadcopter_brain.proportional_position(1, 1, 1)
        command_mock.assert_called_once_with(0.1, 0.9, 0.3)

    @mock.patch('rospy.sleep')
    @mock.patch('quadcopter_brain.QuadcopterBrain.send_rc_command')
    @mock.patch('quadcopter_brain.QuadcopterBrain.proportional_position')
    @mock.patch('quadcopter_brain.QuadcopterBrain.find_landing_site')
    def test_rc_land_on_fiducial(self, site_mock, position_mock, command_mock,
                                 sleep_mock):
        self.landing_site_mock.in_view = True
        self.landing_site_mock.center.position.x = 10
        self.landing_site_mock.center.position.y = 10
        dz_mock = mock.PropertyMock(side_effect=[5, 5, 2, 0.5])
        type(self.landing_site_mock.center.position).z = dz_mock

        site_mock.return_value = (True, None, None)
        self.quadcopter_brain.rc_land_on_fiducial()
        position_calls = [
            mock.call(10, 10, 5), mock.call(10, 10, 2), mock.call(10, 10, 0.5)]
        self.assertEqual(position_calls, position_mock.call_args_list)
        command_calls = [mock.call(0.5, 0.5, 0.25)] * 5
        self.assertEqual(command_calls, command_mock.call_args_list)
        sleep_calls = [
            mock.call(0.1), mock.call(0.1), mock.call(0.1), mock.call(1),
            mock.call(1), mock.call(1), mock.call(1), mock.call(1)]


if __name__ == '__main__':
    unittest.main()
