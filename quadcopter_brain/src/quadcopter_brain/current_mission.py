#!/usr/bin/env python

import rospy

from quadcopter import Quadcopter
from rc_command import RCCommand

def rc_up(q):
    rc_command = RCCommand()
    rc_command.set_throttle(0.55)
    q.send_rc_command(rc_command)

def rc_down(q):
    rc_command = RCCommand()
    rc_command.set_throttle(0.45)
    q.send_rc_command(rc_command)

def forward(q):
    rc_command = RCCommand({'pitch': 0.55})
    q.send_rc_command(rc_command)

def backward(q):
    rc_command = RCCommand({'pitch': 0.45})
    q.send_rc_command(rc_command)

def right(q):
    rc_command = RCCommand({'roll': 0.55})
    q.send_rc_command(rc_command)

def left(q):
    rc_command = RCCommand({'roll': 0.45})
    q.send_rc_command(rc_command)

def still(q):
    rc_command = RCCommand()
    q.send_rc_command(rc_command)

def main():
    q = Quadcopter()
    outside = rospy.get_param("Quadcopter/outside", False)
    if outside:
        q.arm()
    q.launch()

    r = rospy.Rate(0.5)

    print "going forward"
    forward(q)
    r.sleep()
    print "still"
    still(q)
    r.sleep()
    print "going right"
    right(q)
    r.sleep()
    print "still"
    still(q)
    r.sleep()
    print "going backward"
    backward(q)
    r.sleep()
    print "still"
    still(q)
    r.sleep()
    print "going left"
    left(q)
    r.sleep()
    print "still"
    still(q)
    r.sleep()

    print "landing"
    q.land()



    # carl = QuadcopterBrain()

    # # Quadcopter node (carl) must be initialized before get_param will work
    # outside = rospy.get_param("Quadcopter/outside", False)
    # print("In outside mode: %s." % (outside),
    #       "If incorrect, add _outside:=True to the rosrun call")

    # carl.quadcopter.clear_waypoints()
    # print("Sleeping for 3 seconds...")
    # rospy.sleep(3)

    # landing_waypoints = WaypointTools.open_waypoint_file(
    #     "landing_waypoints_2-11-15.json")
    # great_lawn_waypoints = WaypointTools.open_waypoint_file(
    #     "great_lawn_waypoints.json")

    # if outside:
    #     carl.arm()
    # carl.launch()
    # # carl.go_to_waypoints([great_lawn_waypoints["B"],
    # #                       great_lawn_waypoints["A"],
    # #                       great_lawn_waypoints["C"]])
    # # carl.rc_land_on_fiducial()



    # rc

    # carl.land()


def print_position_data(quadcopter):
    print("Position data:")
    print("\tLatitude: %.8f" % quadcopter.current_lat)
    print("\tLongitude: %.8f" % quadcopter.current_long)
    print("\tRelative Altitude: %.2f" % quadcopter.current_rel_alt)
    print("\tAltitude: %.2f" % quadcopter.current_alt)
    print("\tHeading: %.2f" % quadcopter.heading)


if __name__ == '__main__':
    main()
