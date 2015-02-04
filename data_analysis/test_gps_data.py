import unittest

import mock
import numpy

import gps_data
import filtered_pos


class TestLoadGPSData(unittest.TestCase):
    def setUp(self):
        self.data = [filtered_pos.FilteredPos(range(12))] * 10

    def test_extract_gps_coordinates(self):
        gps_coords = gps_data.extract_gps_coordinates(self.data)
        true_gps_coords = numpy.tile(numpy.array([4e-7, 5e-7]), (10, 1))
        self.assertTrue(numpy.array_equal(true_gps_coords, gps_coords))
        self.assertEqual(type(gps_coords), numpy.ndarray)

    def test_extract_gps_times(self):
        gps_times = gps_data.extract_gps_times(self.data)
        true_gps_times = numpy.array([0] * 10)
        self.assertTrue(numpy.array_equal(true_gps_times, gps_times))
        self.assertEqual(type(gps_times), numpy.ndarray)


if __name__ == '__main__':
    unittest.main()
