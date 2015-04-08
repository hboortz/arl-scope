# Suggested filename change - base_station.py or mission_controller.py

from copy import deepcopy
from math import ceil
import datetime

import rospy

from position_tools import PositionTools
from waypoint_tools import WaypointTools
import quadcopter
import landing_site


class QuadcopterBrain(object):
    '''
    High-level quadcopter controller.
    '''
    def __init__(self):
        self.quadcopter = quadcopter.Quadcopter()
        self.landing_site = landing_site.LandingSite()

    def arm(self):
        self.quadcopter.arm()

    def launch(self):
        self.quadcopter.launch()

    def go_to_waypoints(self, waypoint_data, time_to_sleep=15):
        waypoints = [
            WaypointTools.build_waypoint(datum) for datum in waypoint_data]
        for waypoint in waypoints:
            if self.quadcopter.send_waypoint(waypoint):
                rospy.loginfo("Waypoint sent, sleeping %s seconds for arrival",
                              time_to_sleep)
                rospy.sleep(time_to_sleep)
                rospy.loginfo("%s seconds passed, moving on", time_to_sleep)

    def land(self):
        self.quadcopter.land()

    def fly_path(self, waypoint_data):
        self.quadcopter.launch()
        self.go_to_waypoints(waypoint_data)
        self.quadcopter.land()

    def hover_in_place(self):
        waypoint_data = [{"latitude": self.quadcopter.current_lat,
                          "longitude": self.quadcopter.current_long,
                          "altitude": self.quadcopter.current_rel_alt}]
        rospy.loginfo("Sending hover command...")
        self.go_to_waypoints(waypoint_data)
        rospy.loginfo("Hover command sent")

    def go_to_waypoint_given_metered_offset(self, delta_east, delta_north,
                                            dAlt=0, time_to_sleep=15):
        '''
        Given a displacement in meters, this function calculates the desired
        waypoint and tells the quadcopter to go there
        '''
        wp_lat, wp_long = \
            PositionTools.metered_offset(self.quadcopter.current_lat,
                                         self.quadcopter.current_long,
                                         delta_east, delta_north)
        waypoint_data = [{"latitude": wp_lat, "longitude": wp_long,
                          "altitude": self.quadcopter.current_rel_alt + dAlt}]
        rospy.loginfo("Sending relative waypoint...")
        self.go_to_waypoints(waypoint_data, time_to_sleep)
        rospy.loginfo("Relative waypoint sent")

    def find_landing_site_at_waypoints(self, waypoint_data):
        '''
        Takes a list of waypoint ditionaries, goes to each waypoint, and
        tries to find_landing_site at each waypoint
        '''
        for waypoint in waypoint_data:
            self.go_to_waypoints([waypoint])
            found, goal_lat, goal_long = self.find_landing_site(15)
            if found:
                return True, goal_lat, goal_long
        return False, 0, 0

    def find_landing_site(self, wait_seconds=60):
        '''
        Executes a search behavior for the fiducial, return its placement of
        the fiducial it has, in (latitude, longitude) form
        TODO: Make a behavior that takes more data to place the site
        '''
        time_limit = datetime.timedelta(minutes=wait_seconds)
        time_end = datetime.datetime.now() + time_limit
        seen = False
        rospy.loginfo("Searching for landing site, 1 min...")
        while not seen and datetime.datetime.now() < time_end \
                and not rospy.is_shutdown():
            site = deepcopy(self.landing_site)
            seen = site.in_view
            rospy.sleep(0.1)
        if seen:
            rospy.loginfo("Landing site FOUND: %s", str(site.center))
            return (True, ) + \
                site.lat_long(self.quadcopter)  # Returns (bool, int, int)
        else:
            rospy.loginfo("Landing site was NOT FOUND")
            return False, 0, 0

    def land_on_fiducial_simple(self):
        '''
        Looks for the fiducial. When the fiducial is seen a position estimate
        is immediately made and a 1m waypoint is sent at that spot
        '''
        found, goal_lat, goal_long = self.find_landing_site()
        if found:
            waypt = {'latitude': goal_lat,
                     'longitude': goal_long,
                     'altitude': 1.0}
            self.go_to_waypoints([waypt])
        self.land()

    def land_on_fiducial_incremental(self):
        '''
        Averages the position of the fiducial, goes to that spot and steps
        down altitude in discrete steps until low enough, then lands
        '''
        found, _, _ = self.find_landing_site()
        alt = -1.0
        if found:
            goal_lat, goal_long, goal_vertical_dist = \
                self.landing_site.get_average_lat_long(self.quadcopter)
            seen = goal_lat != None
            alt = self.quadcopter.current_rel_alt
            while seen and alt > 1.5:
                if alt > 6.0:
                    next_alt = 5.0
                elif alt > 3.5:
                    next_alt = 2.5
                else:
                    next_alt = 1.0
                waypt = {'latitude': goal_lat,
                         'longitude': goal_long,
                         'altitude': next_alt}
                self.go_to_waypoints([waypt])
                goal_lat, goal_long, goal_vertical_dist = \
                    self.landing_site.get_average_lat_long(self.quadcopter)
                seen = goal_lat != None
                alt = self.quadcopter.current_rel_alt
                seen = goal_lat is not None
        rospy.loginfo("Fiducial found: %s, altitude %f", found, alt)
        self.land()
