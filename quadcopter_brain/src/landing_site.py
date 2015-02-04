#!/usr/bin/env python

import geodesy.utm
import numpy as np
import roslib
import rospy
roslib.load_manifest('ar_pose')
from ar_pose.msg import ARMarkers
from geometry_msgs.msg import Pose, Point

from position_tools import PositionTools


class LandingSite(object):
    def __init__(self):
        '''
        TODO: Variables to implement: orientation, is_upright
        '''
        self.center = Pose()
        in_view = False
        update_sub = rospy.Subscriber('/ar_pose_marker', ARMarkers,
                                      self.on_fiducial_update)

    def on_fiducial_update(self, data):
        '''
        Sets the in_view and center variables when given new data
        '''
        X, Y, Z, visible = self.clean_fiducials(data)
        self.in_view = len(visible) > 0
        if self.in_view:
            self.center = self.find_fiducial_center(X, Y, Z, visible)

    def clean_fiducials(self, data):
        '''
        Takes raw fiducial data (type ARMarkers) and returns list of the
        relative, X, Y, and Z positions of visible fiducials. Also returns a
        list of the visible fiducial ids
        '''
        markers = data.markers
        X = [f.pose.pose.position.x for f in markers]
        Y = [f.pose.pose.position.y for f in markers]
        Z = [f.pose.pose.position.z for f in markers]
        ids = [f.id for f in markers]
        return X, Y, Z, ids

    def find_fiducial_center(self, X, Y, Z, ids):
        '''
        Finds the center of the fiducial, in meters, from the camera
        TODO: Incorporate which fiducials are seen to find the center
        '''
        return Pose(position=Point(x=np.mean(X),
                                   y=np.mean(Y),
                                   z=np.mean(Z)))

    def lat_lon(self, copter):
        '''
        Latitude, longitude of the landing site
        Note: we use (-y) data from the camera because
              the orientation of the reported data has
              (+y) going backwards
        '''
        heading = np.radians(switch_CW_and_CCW(copter.heading))
        rotation = np.array([[np.cos(heading), -np.sin(heading)],
                             [np.sin(heading), np.cos(heading)]])
        relative_site = np.array([[self.center.position.x],
                                  [-self.center.position.y]])
        absolute_site = np.dot(rotation, relative_site)

        return PositionTools.metered_offset(copter.latitude,
                                            copter.longitude,
                                            absolute_site[0][0],
                                            absolute_site[1][0])


def switch_CW_and_CCW(aircraft_heading):
    '''
    Converts a heading from clockwise to counterclockwise
    '''
    math_heading = -(aircraft_heading - 360)
    return math_heading
