import rospy

from quadcopter_brain import QuadcopterBrain
from waypoint_tools import WaypointTools


def main():
    outside = rospy.get_param("quadcopter_brain/outside", False)
    print("In outside mode: %s." % (outside),
          "If incorrect, add _outside:=True to rosrun")

    carl = QuadcopterBrain()
    carl.quadcopter.clear_waypoints()
    print("Sleeping for 3 seconds...")
    rospy.sleep(3)

    great_lawn_waypoints = WaypointTools.open_waypoint_file(
        "waypoint_data/great_lawn_waypoints.json")

    if outside:
        carl.arm()
    carl.fly_path([great_lawn_waypoints["A"], great_lawn_waypoints["B"]])


def print_position_data(quadcopter):
    print("Position data:")
    print("\tLatitude: %.8f" % quadcopter.current_lat)
    print("\tLongitude: %.8f" % quadcopter.current_long)
    print("\tRelative Altitude: %.2f" % quadcopter.current_rel_alt)
    print("\tAltitude: %.2f" % quadcopter.current_alt)
    print("\tHeading: %.2f" % quadcopter.heading)

if __name__ == '__main__':
    main()
