# Suggested filename change - base_station.py or mission_controller.py

from copy import deepcopy
from math import ceil
import datetime

import rospy

from position_tools import PositionTools
from waypoint_tools import WaypointTools
from landing_site import LandingSite
import quadcopter


class QuadcopterBrain(object):
    '''
    High-level quadcopter controller.
    '''
    def __init__(self):
        self.quadcopter = quadcopter.Quadcopter()
        self.landing_site = LandingSite()

    def arm(self):
        self.quadcopter.arm()

    def launch(self):
        self.quadcopter.launch()

    def go_to_waypoints(self, waypoint_data, time_to_sleep=15):
        waypoints = [
            WaypointTools.build_waypoint(datum) for datum in waypoint_data]
        for waypoint in waypoints:
            if self.quadcopter.send_waypoint(waypoint):
                print("Waypoint sent, sleeping %s seconds for arrival"
                      % time_to_sleep)
                rospy.sleep(time_to_sleep)
                print("%s seconds passed, moving on" % time_to_sleep)
                # self.check_reached_waypoint(waypoint)

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
        print("Sending hover command...")
        self.go_to_waypoints(waypoint_data)
        print("Hover command sent")

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
        print("Sending relative waypoint...")
        self.go_to_waypoints(waypoint_data, time_to_sleep)
        print("Relative waypoint sent")

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
        wp_lat = PositionTools.mavlink_to_gps(waypoint.latitude)
        wp_long = PositionTools.mavlink_to_gps(waypoint.longitude)

        error_margin = 3  # in meters
        print "Checking reached:"
        try:
            _, _, dist_from_waypoint = \
                PositionTools.lat_long_diff(self.current_lat,
                                            self.current_long,
                                            wp_lat,
                                            wp_long)
            print "Distance to waypoint: " + str(dist_from_waypoint)
            print "Current pos: %s, %s" % (self.current_lat, self.current_long)
            return dist_from_waypoint < error_margin
        except AttributeError:  # if haven't gotten current position data
            return False

    def find_landing_site(self):
        '''
        Executes a search behavior for the fiducial, return its placement of
        the fiducial it has, in (latitude, longitude) form
        TODO: Make a behavior that takes more data to place the site
        '''
        time_limit = datetime.timedelta(minutes=1)
        time_end = datetime.datetime.now() + time_limit
        seen = False
        print "Searching for landing site..."
        while not seen and datetime.datetime.now() < time_end \
                and not rospy.is_shutdown():
            site = deepcopy(self.landing_site)
            seen = site.in_view
            rospy.sleep(0.1)
        if seen:
            print "Landing site FOUND: ", site.center
            return (True, ) + \
                site.lat_long(self.quadcopter)  # Returns (bool, int, int)
        else:
            print "Landing site was NOT FOUND"
            return False, 0, 0

    def land_on_fiducial_simple(self):
        found, goal_lat, goal_long = self.find_landing_site()
        if found:
            waypt = {'latitude': goal_lat,
                     'longitude': goal_long,
                     'altitude': 1.0}
            self.go_to_waypoints([waypt])
        self.land()

    def land_on_fiducial_incremental(self):
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
                self.go_to_waypoints([waypt], time_to_sleep=8)
                goal_lat, goal_long, goal_vertical_dist = \
                    self.landing_site.get_average_lat_long(self.quadcopter)
                seen = goal_lat != None
                alt = self.quadcopter.current_rel_alt

        print("Fiducial found: %s, altitude %f" % (found, alt))
        self.land()
