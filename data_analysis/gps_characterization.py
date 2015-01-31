import os

import gps_data
import gps_metrics 

def main():
    user = os.environ["USER"]
    filename = ("/home/" + user + "/catkin_ws/src/arl-scope/bagfiles"
                "/carl_greatlawn_bottomleftconner_2015-01-28-15-32-40.csv")
    data = gps_data.load_gps_data(filename)
    gps_coordinates = gps_data.extract_gps_coordinates(data)
    gps_times = gps_data.extract_gps_times(data)

    print gps_metrics.center_of_gravity(gps_coordinates)
    print gps_metrics.precision(gps_coordinates)
    print gps_metrics.average_speed(gps_times, gps_coordinates)



if __name__ == '__main__':
    main()
