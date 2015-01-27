#!/usr/bin/env python

from copy import deepcopy
import geodesy.utm
from math import radians, sin, cos
import numpy as np
import roslib
import rospy

roslib.load_manifest('ar_pose')
from ar_pose.msg import ARMarkers
from geometry_msgs.msg import Pose, Point


class LandingSite(object):
    def __init__(self):
        self.center = Pose()
        in_view = False
        orientation = 0.0
        is_upright = True
        update_sub = rospy.Subscriber('/ar_pose_marker',
                                      ARMarkers,
                                      self.on_fiducial_update)

    def on_fiducial_update(data):
        markers = data.markers
        visible = []
        sums = [0, 0, 0]
        count = 0
        for fiducial in markers:
            visible.append(fiducial.id)
            count += 1
            fpose = fiducial.pose.pose
            sums[0] += fpose.x
            sums[1] += fpose.y
            sums[2] += fpose.z
        self.in_view = len(visible) > 0
        self.center = Pose(position=Point(x=sums[0]/count,
                                          y=sums[1]/count,
                                          z=sums[2]/count))
        print "Landing site: ", self.center

    def latlon(self, copter):
        '''
        Latitude, longitude of the landing site
        Note: we use (-y) data from the camera because
              the orientation of the reported data has
              (+y) going backwards
        '''
        heading = radians(air_to_math(copter.heading))
        print "copter.heading:", copter.heading
        print "heading:", heading
        rotation = np.array([[cos(heading), -sin(heading)],
                             [sin(heading), cos(heading)]])
        relative_site = np.array([[self.center.position.x], 
                                  [-self.center.position.y]])
        print "relative_site:", relative_site
        absolute_site = np.dot(rotation, relative_site)
        print "absolute_site:", absolute_site
        copter_utm = geodesy.utm.fromLatLong(copter.latitude,
                                             copter.longitude)
        site_utm = deepcopy(copter_utm)
        site_utm.easting += absolute_site[0][0]
        site_utm.northing += absolute_site[1][0]
        site_latlon = site_utm.toMsg()
        return site_latlon.latitude, site_latlon.longitude


def air_to_math(aircraft_heading):
    '''
    Converts a heading from clockwise to counterclockwise
    '''
    math_heading = -(aircraft_heading - 360)
    return math_heading
