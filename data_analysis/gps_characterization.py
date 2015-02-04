import os

import geodesy.utm

import gps_data
import gps_metrics
import plotting
import matplotlib.pyplot as plt


def print_metrics(gps_coordinates, gps_times):
    print gps_metrics.center_of_gravity(gps_coordinates)
    print gps_metrics.precision(gps_coordinates)
    print gps_metrics.average_speed(gps_times, gps_coordinates)


def get_filename():
    user = os.environ["USER"]
    filename = ("/home/" + user + "/catkin_ws/src/arl-scope/bagfiles"
                "/carl_nearpostandroad_2015-01-28-15-51-22.csv")
    return filename


def plot_gps_coordinates_in_meters(measured_gps, true_gps):
    measured_gps = lat_lon_to_meters(measured_gps)
    true_gps = lat_lon_to_meters([true_gps])

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


def lat_lon_to_meters(gps_points):
    utm_points = [geodesy.utm.fromLatLong(lat, lon) for lat, lon in gps_points]
    x_metered_points = [point.northing for point in utm_points]
    y_metered_points = [point.easting for point in utm_points]

    return (x_metered_points, y_metered_points)


def main():
    data = gps_data.load_gps_data(get_filename())
    gps_coordinates = gps_data.extract_gps_coordinates(data)
    gps_times = gps_data.extract_gps_times(data)
    #print_metrics(gps_coordinates, gps_times)
    plot_gps_coordinates_in_meters(gps_coordinates, (42.292050, -71.262735))


if __name__ == '__main__':
    main()
