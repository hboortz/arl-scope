#!/usr/bin/env python

from copy import deepcopy

import geodesy.utm
import numpy as np
import roslib
import rospy
roslib.load_manifest('ar_pose')
from ar_pose.msg import ARMarkers
from geometry_msgs.msg import Pose, Point


class LandingSite(object):
    def __init__(self):
        '''
        Variables to implement:
        orientation
        is_upright
        '''
        self.center = Pose()
        in_view = False
        update_sub = rospy.Subscriber('/ar_pose_marker',
                                      ARMarkers,
                                      self.on_fiducial_update)

    def clean_fiducials(data):
        markers = data.markers
        X = [f.pose.pose.position.x for f in markers]
        Y = [f.pose.pose.position.y for f in markers]
        Z = [f.pose.pose.position.z for f in markers]
        ids = [f.id for f in markers]
        return X, Y, Z, ids

    def find_fiducial_center(X, Y, Z, ids):
        '''
        Finds the center of the fiducial, in meters, from the camera
        TODO: Incorporate which fiducials are seen to find the center
        '''
        return Pose(position=Point(x=np.mean(X),
                                   y=np.mean(Y),
                                   z=np.mean(Z)))

    def on_fiducial_update(self,data):
        X, Y, Z, visible = clean_fiducials(data)
        self.in_view = len(visible) > 0
        if visible:
            self.center = find_fiducial_center(X, Y, Z, visible)

    def lat_lon(self, copter):
        '''
        Latitude, longitude of the landing site
        Note: we use (-y) data from the camera because
              the orientation of the reported data has
              (+y) going backwards
        '''
        heading = np.radians(air_to_math(copter.heading))
        rotation = np.array([[np.cos(heading), -np.sin(heading)],
                             [np.sin(heading), np.cos(heading)]])
        relative_site = np.array([[self.center.position.x], 
                                  [-self.center.position.y]])
        absolute_site = np.dot(rotation, relative_site)
        copter_utm = geodesy.utm.fromLatLong(copter.latitude,
                                              copter.longitude)
        site_utm = deepcopy(copter_utm)
        site_utm.easting += absolute_site[0][0]
        site_utm.northing += absolute_site[1][0]
        site_lat_lon = site_utm.toMsg()
        return site_lat_lon.latitude, site_lat_lon.longitude


def air_to_math(aircraft_heading):
    '''
    Converts a heading from clockwise to counterclockwise
    '''
    math_heading = -(aircraft_heading - 360)
    return math_heading
