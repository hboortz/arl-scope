#!/usr/bin/env python
# 10/22/2014
# Charles O. Goddard

import rospy
import tf

import roscopter


class Carl(object):
    '''
    High-level quadcopter controller.
    '''

    def on_position_update(self, data):
        pass

    def __init__(self):
        # Set up position listener
        rospy.Subscriber("filtered_pos",
                         roscopter.msg.FilteredPosition,
                         self.on_position_update)

        # Create service proxies
        self.command  = rospy.ServiceProxy('command',
                                          roscopter.srv.APMCommand)
        self.waypoint = rospy.ServiceProxy('waypoint',
                                           roscopter.srv.SendWaypoint)
        self.land     = rospy.ServiceProxy('land', Empty)


if __name__ == '__main__':
    rospy.init_node("carl")
