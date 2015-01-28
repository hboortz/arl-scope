import unittest

import mock
import numpy

import gps_characterization


class TestGPSCharacterization(unittest.TestCase):
    def test_center_of_gravity(self):
        p1 = numpy.array([1, 0])
        p2 = numpy.array([0, 1])
        p3 = numpy.array([-1, 0])
        p4 = numpy.array([0, -1])
        points = numpy.array([p1, p2, p3, p4])

        cog = gps_characterization.center_of_gravity(points)
        self.assertTrue(numpy.allclose(cog, numpy.array([0, 0])))

    def test_euclidean_distance(self):
        p1 = numpy.array([0, 0])
        p2 = numpy.array([1, 1])

        distance = gps_characterization.euclidean_distance(p1, p2)
        self.assertAlmostEqual(distance, numpy.sqrt(2))

    @mock.patch("gps_characterization.center_of_gravity")
    def test_precision(self, center_of_gravity_mock):
        p1 = numpy.array([1, 0])
        p2 = numpy.array([0, 2])
        p3 = numpy.array([-1, 0])
        p4 = numpy.array([0, -2])
        points = numpy.array([p1, p2, p3, p4])

        center_of_gravity_mock.return_value = numpy.array([0, 0])
        precision = gps_characterization.precision(points)
        self.assertAlmostEqual(1.5, precision)

    def test_moving_average(self):
        points = numpy.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

        true_moving_averages = numpy.array([3, 4, 5, 6, 7, 8])
        moving_averages = gps_characterization.simple_moving_average(points)
        self.assertTrue(numpy.allclose(moving_averages, true_moving_averages))

    @mock.patch("gps_characterization.euclidean_distance")
    def test_speeds(self, euclidean_distance_mock):
        times = numpy.array([1, 3, 4, 5, 7, 8, 10, 11, 12, 14, 15])
        data = numpy.tile(numpy.array([1, 1]), (11, 1))
        euclidean_distance_mock.side_effect = [2, 1, 2, 2, 1, 2, 1, 2, 2, 1]

        true_speeds = numpy.array([1, 1, 2, 1, 1, 1, 1, 2, 1, 1])
        speeds = gps_characterization.speeds(times, data)

        self.assertTrue(numpy.allclose(true_speeds, speeds))

        


if __name__ == "__main__":
    unittest.main()