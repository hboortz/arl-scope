# Suggested filename change - base_station.py or mission_controller.py

from copy import deepcopy
import datetime
import time
import math

import rospy

from flight_error import FlightError
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

    def rc_land_on_fiducial(self):
        dz = self.landing_site.center.position.z
        while dz > 1:
            dx = self.landing_site.center.position.x
            dy = self.landing_site.center.position.y
            rc_proportionally_navigate(dx, dy, dz)

        self.land()

    def rc_proportionally_navigate(dx, dy, dz):
        pass

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
            alt = self.quadcopter.current_rel_alt
            seen = True
            while seen and alt > 4.0:
                goal_lat, goal_long, goal_vertical_dist = \
                    self.landing_site.get_average_lat_long(self.quadcopter)
                waypt = {'latitude': goal_lat,
                         'longitude': goal_long,
                         'altitude': alt - 2.0}
                self.go_to_waypoints([waypt], time_to_sleep=8)
                alt = self.quadcopter.current_rel_alt
                seen = goal_lat is not None
        rospy.loginfo("Fiducial found: %s, altitude %f", found, alt)
        self.land()
