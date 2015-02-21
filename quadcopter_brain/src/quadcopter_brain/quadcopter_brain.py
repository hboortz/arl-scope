#!/usr/bin/env python

import rospy

from position_tools import PositionTools
from waypoint_tools import WaypointTools


class QuadcopterBrain(object):
    '''
    High-level quadcopter controller.
    '''
    def __init__(self):
        self.quadcopter = Quadcopter()

    def go_to_waypoints(self, waypoint_data):
        waypoints = [
            WaypointTools.build_waypoint(datum) for datum in waypoint_data]
        for waypoint in waypoints:
            self.quadcopter.send_waypoint(waypoint)

    def fly_path(self, waypoint_data):
        self.quadcopter.launch()
        self.go_to_waypoints(waypoint_data)
        self.quadcopter.land()

    def check_reached_waypoint(self, waypoint):
        wait_time = 0
        while not self.has_reached_waypoint and wait_time < 50:
            rospy.sleep(5)
            wait_time += 5
            print "--> Traveling to waypoint for %d seconds" % (wait_time)
            print "--> Current position is %d, %d" % (self.current_lat,
                                                      self.current_long)
        if wait_time < 50:  # successfully reached
            rospy.sleep(5)  # stay at waypoint for a few seconds
            return "Reached waypoint"
        else:
            return "Failed to reach waypoint"

    def has_reached_waypoint(self, waypoint):
        error_margin = 3  # in meters
        try:
            _, _, dist_from_waypoint = \
                PositionTools.lat_lon_diff(self.quadcopter.current_lat,
                                           self.quadcopter.current_long,
                                           waypoint.latitude,
                                           waypoint.longitude)
            return dist_from_waypoint < error_margin
        except AttributeError:  # if haven't gotten current position data
            return False
