import os
import sys
import unittest
sys.path.append(os.path.relpath('.../../quadcopter_brain/src/'))

import mock
import numpy

import gps_metrics
from position_tools import PositionTools


class TestGPSCharacterization(unittest.TestCase):
    def test_center_of_gravity(self):
        p1 = numpy.array([1, 0])
        p2 = numpy.array([0, 1])
        p3 = numpy.array([-1, 0])
        p4 = numpy.array([0, -1])
        points = numpy.array([p1, p2, p3, p4])

        cog = gps_metrics.center_of_gravity(points)
        self.assertTrue(numpy.allclose(cog, numpy.array([0, 0])))

    @mock.patch("gps_metrics.center_of_gravity")
    @mock.patch("position_tools.PositionTools")
    def test_precision(self, position_tools_mock, center_of_gravity_mock):
        p1 = numpy.array([1, 0])
        p2 = numpy.array([0, 2])
        p3 = numpy.array([-1, 0])
        p4 = numpy.array([0, -2])
        points = numpy.array([p1, p2, p3, p4])

        center_of_gravity_mock.return_value = numpy.array([0, 0])
        position_tools_mock.lat_lon_diff.side_effect = [
            (1, 1, 1), (2, 2, 2), (1, 1, 1), (2, 2, 2)
        ]

        precision = gps_metrics.precision(points)
        self.assertAlmostEqual(1.5, precision)

    @mock.patch("gps_metrics.euclidean_distance")
    def test_speeds(self, euclidean_distance_mock):
        times = numpy.array([1, 3, 4, 5, 7, 8, 10, 11, 12, 14, 15])
        data = numpy.tile(numpy.array([1, 1]), (11, 1))
        euclidean_distance_mock.side_effect = [2, 1, 2, 2, 1, 2, 1, 2, 2, 1]

        true_speeds = numpy.array([1, 1, 2, 1, 1, 1, 1, 2, 1, 1])
        speeds = gps_metrics.speeds(times, data)

        self.assertTrue(numpy.allclose(true_speeds, speeds))

    @mock.patch("gps_metrics.speeds")
    def test_average_speed(self, speeds_mock):
        times = numpy.array([1, 3, 4, 5, 7, 8, 10, 11, 12, 14, 15])
        data = numpy.tile(numpy.array([1, 1]), (11, 1))
        speeds_mock.return_value = range(10)

        average_speed = gps_metrics.average_speed(times, data)
        self.assertAlmostEqual(4.5, average_speed)


if __name__ == "__main__":
    unittest.main()
