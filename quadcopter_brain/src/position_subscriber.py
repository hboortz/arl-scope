#!/usr/bin/env python

import rospy

import roscopter
import roscopter.msg
import roscopter.srv

class CurrentPosition(object):
    def __init__(self):
        print "MADE OBJECT"
        rospy.Subscriber("/filtered_pos", roscopter.msg.FilteredPosition,
                         self.callback)
        print "MADE SUB"
        rospy.spin()

    def __str__(self):
        self.latitude

    def callback(self, data):
        print "REL_ALT: ", data.relative_altitude
        print "---------------------"
        self.relative_altitude = data.relative_altitude
        

if __name__ == '__main__':
    print "RAWR"
    carl_pos = CurrentPosition()
