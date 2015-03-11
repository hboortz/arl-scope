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
    carl.go_to_waypoints([great_lawn_waypoints["B"],
                          great_lawn_waypoints["A"],
                          great_lawn_waypoints["C"]])
    carl.land()


def print_position_data(quadcopter):
    print("Position data:")
    location = quadcopter.last_known_pose()
    print("\tLatitude: %.8f" % location.latitude)
    print("\tLongitude: %.8f" % location.longitude)
    print("\tRelative Altitude: %.2f" % location.relative_altitude)
    print("\tAltitude: %.2f" % location.altitude)
    print("\tHeading: %.2f" % location.heading)


if __name__ == '__main__':
    main()
