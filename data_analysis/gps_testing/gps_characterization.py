import os

import geodesy.utm
import matplotlib.pyplot as plt

import gps_data
import gps_metrics
import plotting
from quadcopter_brain import position_tools


def print_metrics(gps_coordinates, gps_times):
    print gps_metrics.center_of_gravity(gps_coordinates)
    print gps_metrics.precision(gps_coordinates)
    print gps_metrics.average_speed(gps_times, gps_coordinates)


def get_filepath(filename):
    user = os.environ["USER"]
    filepath = (
        "/home/%s/catkin_ws/src/arl-scope/bagfiles/%s" % (user, filename)
    )
    return filepath


def plot_gps_coordinates_in_meters(measured_gps, true_gps):
    measured_gps = position_tools.PositionTools.lat_lon_to_meters(measured_gps)
    true_gps = position_tools.PositionTools.lat_lon_to_meters([true_gps])

    fig, ax = plt.subplots(1, 1)
    plotting.plot_xy_coordinates(
        ax, measured_gps[0], measured_gps[1], {'marker': 'o', 'color': 'green'}
    )
    plotting.plot_xy_coordinates(
        ax, true_gps[0], true_gps[1],
        {'marker': 'o', 'color': 'red', 'markersize': 16}
    )
    plotting.label_axes(
        ax, 'Raised Stationary GPS Test (2)', 'Latitude (m)', 'Longitude (m)'
    )
    plt.show()


def main():
    filepath = get_filepath("hover_data_2015-02-11_last_30s_bad.csv")
    data = gps_data.load_gps_data(filepath)
    gps_coordinates = gps_data.extract_gps_coordinates(data)
    gps_times = gps_data.extract_gps_times(data)
    print_metrics(gps_coordinates, gps_times)
    plot_gps_coordinates_in_meters(gps_coordinates, (42.292101, -71.262690))


if __name__ == '__main__':
    main()
