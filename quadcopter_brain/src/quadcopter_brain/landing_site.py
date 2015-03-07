#!/usr/bin/env python

import numpy as np
import geodesy.utm
import roslib
import rospy
roslib.load_manifest('ar_pose')
from ar_pose.msg import ARMarkers
from geometry_msgs.msg import Pose, Point

from position_tools import PositionTools


class LandingSite(object):
    def __init__(self):
        # TODO: Variables to implement: orientation, is_upright
        self.center = Pose()
        in_view = False
        update_sub = rospy.Subscriber('/ar_pose_marker', ARMarkers,
                                      self.on_fiducial_update)

    def on_fiducial_update(self, data):
        '''
        Sets the in_view and center variables when given new data
        '''
        x_coords, y_coords, z_coords, visible = self.clean_fiducials(data)
        self.in_view = len(visible) > 0
        if self.in_view:
            self.center = self.find_fiducial_center(x_coords, y_coords,
                                                    z_coords, visible)

    def clean_fiducials(self, data):
        '''
        Takes raw fiducial data (type ARMarkers) and returns list of the
        relative, x_coords, y_coords, and z_coords positions of visible
        fiducials. Also returns a list of the visible fiducial ids
        '''
        markers = data.markers
        x_coords = [f.pose.pose.position.x for f in markers]
        y_coords = [f.pose.pose.position.y for f in markers]
        z_coords = [f.pose.pose.position.z for f in markers]
        ids = [f.id for f in markers]
        return x_coords, y_coords, z_coords, ids

    def find_fiducial_center(self, x_coords, y_coords, z_coords, ids):
        '''
        Finds the center of the fiducial, in meters, from the camera
        TODO: Incorporate which fiducials are seen to find the center
        '''
        return Pose(position=Point(x=np.mean(x_coords),
                                   y=np.mean(y_coords),
                                   z=np.mean(z_coords)))

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

        return PositionTools.metered_offset(copter.current_lat,
                                            copter.current_long,
                                            absolute_site[0][0],
                                            absolute_site[1][0])

    def error_vector(self, copter):
        '''
            position of landing pad reletive to quadcopter
        '''
        return self.center


def switch_CW_and_CCW(aircraft_heading):
    '''
    Converts a heading from clockwise to counterclockwise
    '''
    math_heading = 360 - aircraft_heading
    return math_heading
