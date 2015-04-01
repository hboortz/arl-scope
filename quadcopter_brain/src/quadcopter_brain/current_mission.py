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
    rc_command = RCCommand({'pitch': 0.6})
    q.send_rc_command(rc_command)

def backward(q):
    rc_command = RCCommand({'pitch': 0.4})
    q.send_rc_command(rc_command)

def right(q):
    rc_command = RCCommand({'roll': 0.6})
    q.send_rc_command(rc_command)

def left(q):
    rc_command = RCCommand({'roll': 0.4})
    q.send_rc_command(rc_command)

def still(q):
    rc_command = RCCommand()
    q.send_rc_command(rc_command)

def throttle_up(q):
    rc_command = RCCommand({"throttle": 0.6})
    q.send_rc_command(rc_command)

def throttle_down(q):
    rc_command = RCCommand({"throttle": 0.4})
    q.send_rc_command(rc_command)

def main():

    import time

    q = Quadcopter()
    outside = rospy.get_param("Quadcopter/outside", False)
    if outside:
        q.arm()
    q.launch()

    r = rospy.Rate(0.5)

    print "forward"
    backward(q)
    r.sleep()
    print "still"
    still(q)
    time.sleep(2)

    print "right"
    right(q)
    r.sleep()
    print "still"
    still(q)
    time.sleep(2)

    print "backward"
    forward(q)
    r.sleep()
    print "still"
    still(q)
    time.sleep(2)

    print "left"
    left(q)
    r.sleep()
    print "still"
    still(q)
    time.sleep(2)
   

    q.return_rc_control()


def print_position_data(quadcopter):
    print("Position data:")
    print("\tLatitude: %.8f" % quadcopter.current_lat)
    print("\tLongitude: %.8f" % quadcopter.current_long)
    print("\tRelative Altitude: %.2f" % quadcopter.current_rel_alt)
    print("\tAltitude: %.2f" % quadcopter.current_alt)
    print("\tHeading: %.2f" % quadcopter.heading)


if __name__ == '__main__':
    main()
