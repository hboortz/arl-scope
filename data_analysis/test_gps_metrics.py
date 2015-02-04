import unittest

import mock
import numpy

import gps_metrics


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
    @mock.patch("gps_metrics.latlon_diff")
    def test_precision(self, latlon_diff_mock, center_of_gravity_mock):
        p1 = numpy.array([1, 0])
        p2 = numpy.array([0, 2])
        p3 = numpy.array([-1, 0])
        p4 = numpy.array([0, -2])
        points = numpy.array([p1, p2, p3, p4])

        latlon_diff_mock.side_effect = [1, 2, 1, 2]
        center_of_gravity_mock.return_value = numpy.array([0, 0])
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
