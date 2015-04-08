# Suggested filename change - base_station.py or mission_controller.py

from copy import deepcopy
import datetime
import time
import math

import rospy

from flight_error import FlightError
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
                # print("Waypoint sent, sleeping %d seconds for arrival"
                #      % time_to_sleep)
                # rospy.sleep(time_to_sleep)
                # print("%s seconds passed, moving on" % time_to_sleep)
                print("Waypoint sent.")
                print self.check_reached_waypoint(waypoint)

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

    def check_reached_waypoint(self, waypoint, max_wait_time=60, wait_time=0):
        '''
        Checks if the quadcopter reaches the waypoint within the specified
        max_wait_time (seconds)
        '''
        while (not self.has_reached_waypoint(waypoint)) and \
                wait_time < max_wait_time:
            time.sleep(5)
            wait_time += 5
            print "--> Traveling to waypoint for %d seconds" % (wait_time)
            print "--> Current pos: %f, %f" % (self.quadcopter.current_lat,
                                               self.quadcopter.current_long)
        if wait_time < max_wait_time:  # successfully reached
            time.sleep(5)  # stay at waypoint for a few seconds
            return "Reached waypoint"
        else:
            return "failed to reach waypoint"
            #return self.waypoint_timeout_choice(waypoint, wait_time)

    def waypoint_timeout_choice(self, waypoint, curr_wait_time):
        print "TIMEOUT: Traveling to waypoint for %d sec." % (curr_wait_time)
        opt1 = "\t 1 - Continue traveling to waypoint\n"
        opt2 = "\t 2 - Continue to next command \n"
        opt3 = "\t 3 - Terminate \n"
        msg = "\t Choose an option number:\n%s%s%s>>> " % (opt1, opt2, opt3)
        while not rospy.is_shutdown():
            time.sleep(0.1)  # there's some weird not getting input thing
            try:
                choice = raw_input(msg)
                if choice == '1':
                    print "Continuing toward waypoint."
                    return self.check_reached_waypoint(
                        waypoint,
                        max_wait_time=curr_wait_time*2,
                        wait_time=curr_wait_time)
                elif choice == '2':
                    return "Failed to reach waypoint. \
                        Continuing to next command"
                elif choice == '3':
                    raise FlightError("Timeout going to waypoint", self)
                else:
                    raise SyntaxError  # this gets caught in the except
            except (SyntaxError, EOFError, NameError):
                print "Invalid Choice."
                msg = "Enter either 1, 2, or 3. \n>>> "

    def has_reached_waypoint(self, waypoint, xy_error_margin=3,
                             alt_error_margin=1):
        """ Waypoint is roscopter waypoint type
            error margins are in meters

            returns boolean of whether position is within error margins"""
        try:
            waypoint_lat = PositionTools.mavlink_to_gps(waypoint.latitude)
            waypoint_long = PositionTools.mavlink_to_gps(waypoint.longitude)
            waypoint_alt = waypoint.altitude / 1000.0  # mm to m
            _, _, dist = PositionTools.lat_long_diff(
                self.quadcopter.current_lat,
                self.quadcopter.current_long,
                waypoint_lat,
                waypoint_long)
            alt_diff = math.fabs(self.quadcopter.current_rel_alt - waypoint_alt)
            res = dist < xy_error_margin and alt_diff < alt_error_margin
            print "Distance away: lat/long: %.2f, alt: %.2f" % (dist, alt_diff)
            return res
        except AttributeError:  # if haven't gotten current position data
            return False

    def fly_path(self, waypoint_data):
        self.launch()
        self.go_to_waypoints(waypoint_data)
        self.land()

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
        while not seen and datetime.datetime.now() < time_end and \
                not rospy.is_shutdown():
            site = deepcopy(self.landing_site)
            seen = site.in_view
            rospy.sleep(0.1)
        if seen:
            print "Landing site FOUND: ", site.center
            return (True,) + \
                site.lat_long(self.quadcopter)  # Returns (bool, int, int)
        else:
            print "Landing site was NOT FOUND"
            return False, 0, 0  # Returns (bool, int, int)

    def land_on_fiducial_simple(self):
        found, goal_lat, goal_long = self.find_landing_site()
        if found:
            waypt = {'latitude': goal_lat,
                     'longitude': goal_long,
                     'altitude': 1.0}
            self.go_to_waypoints([waypt])
        self.land()
