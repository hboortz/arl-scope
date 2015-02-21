import rospy

from quadcopter_brain import QuadcopterBrain
from waypoint_tools import WaypointTools


def main():
    outside = rospy.get_param("outside", False)
    carl = QuadcopterBrain()
    carl.quadcopter.clear_waypoints()
    print "Sleeping for 3 seconds..."
    rospy.sleep(3)
    great_lawn_waypoints = WaypointTools.open_waypoint_file(
        "waypoint_data/great_lawn_waypoints.json")
    if outside:
        carl.arm()
    carl.fly_path([great_lawn_waypoints["A"], great_lawn_waypoints["B"]])


if __name__ == '__main__':
    main()
