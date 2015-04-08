#!/usr/bin/env python

import rospy

from quadcopter import Quadcopter
from rc_command import RCCommand


def backward(q):
    rc_command = RCCommand({'pitch': 0.9})
    q.send_rc_command(rc_command)


def forward(q):
    rc_command = RCCommand({'pitch': 0.1})
    q.send_rc_command(rc_command)


def right(q):
    rc_command = RCCommand({'roll': 0.9})
    q.send_rc_command(rc_command)


def left(q):
    rc_command = RCCommand({'roll': 0.1})
    q.send_rc_command(rc_command)


def still(q):
    rc_command = RCCommand()
    q.send_rc_command(rc_command)


def throttle_up(q):
    rc_command = RCCommand({"throttle": 0.8})
    q.send_rc_command(rc_command)


def throttle_down(q):
    rc_command = RCCommand({"throttle": 0.2})
    q.send_rc_command(rc_command)


def rc_square_dance():
    import time

    q = Quadcopter()
    outside = rospy.get_param("Quadcopter/outside", False)
    if outside:
        q.arm()
    q.launch()

    print "forward"
    forward(q)
    time.sleep(2)
    print "still"
    still(q)
    time.sleep(2)

    print "right"
    right(q)
    time.sleep(2)
    print "still"
    still(q)
    time.sleep(2)

    print "backward"
    backward(q)
    time.sleep(2)
    print "still"
    still(q)
    time.sleep(2)

    print "left"
    left(q)
    time.sleep(2)
    print "still"
    still(q)
    time.sleep(2)

    throttle_down(q)
    time.sleep(20)


def print_position_data(quadcopter):
    rospy.loginfo("Position data:")
    rospy.loginfo("\tLatitude: %.8f" % quadcopter.current_lat)
    rospy.loginfo("\tLongitude: %.8f" % quadcopter.current_long)
    rospy.loginfo("\tRelative Altitude: %.2f" % quadcopter.current_rel_alt)
    rospy.loginfo("\tAltitude: %.2f" % quadcopter.current_alt)
    rospy.loginfo("\tHeading: %.2f" % quadcopter.heading)


def main():
    carl = QuadcopterBrain()

    # Quadcopter node (carl) must be initialized before get_param will work
    outside = rospy.get_param("Quadcopter/outside", False)
    rospy.loginfo("In outside mode: %s.", outside)
    rospy.loginfo("If incorrect, add _outside:=True to the rosrun call")

    carl.quadcopter.clear_waypoints()
    rospy.loginfo("Sleeping for 3 seconds...")
    rospy.sleep(3)

    great_lawn_waypoints = WaypointTools.open_waypoint_file(
        "great_lawn_waypoints.json")

    if outside:
        carl.arm()
    carl.launch()
    carl.go_to_waypoints([great_lawn_waypoints['A'],
                          great_lawn_waypoints['B5'],
                          great_lawn_waypoints['C10']])
    carl.land()


if __name__ == '__main__':
    main()
