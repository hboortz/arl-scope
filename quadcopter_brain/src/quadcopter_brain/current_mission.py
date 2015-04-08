#!/usr/bin/env python

import rospy

from quadcopter_brain import QuadcopterBrain
from waypoint_tools import WaypointTools


def main():
    carl = QuadcopterBrain()

    # Quadcopter node (carl) must be initialized before get_param will work
    outside = rospy.get_param("Quadcopter/outside", False)
    print("In outside mode: %s." % (outside),
          "If incorrect, add _outside:=True to the rosrun call")

    carl.quadcopter.clear_waypoints()
    print("Sleeping for 3 seconds...")
    rospy.sleep(3)

    great_lawn_waypoints = WaypointTools.open_waypoint_file(
        "great_lawn_waypoints.json")

    if outside:
        carl.arm()
    carl.launch()
    carl.go_to_waypoint_given_metered_offset(0.0, 0.0, -4.0)
    for i in range(2):
        carl.go_to_waypoint_given_metered_offset(3.0, 0.0, time_to_sleep=10)
        carl.go_to_waypoint_given_metered_offset(0.0, 3.0, time_to_sleep=10)
        carl.go_to_waypoint_given_metered_offset(-3.0, 0.0, time_to_sleep=10)
        carl.go_to_waypoint_given_metered_offset(0.0, -3.0, time_to_sleep=10)
    carl.land()


def print_position_data(quadcopter):
    print("Position data:")
    print("\tLatitude: %.8f" % quadcopter.current_lat)
    print("\tLongitude: %.8f" % quadcopter.current_long)
    print("\tRelative Altitude: %.2f" % quadcopter.current_rel_alt)
    print("\tAltitude: %.2f" % quadcopter.current_alt)
    print("\tHeading: %.2f" % quadcopter.heading)


if __name__ == '__main__':
    main()
