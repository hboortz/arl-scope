# Suggested filename change - base_station.py or mission_controller.py

from copy import deepcopy
from math import ceil
import datetime

import rospy
import numpy

from position_tools import PositionTools
from waypoint_tools import WaypointTools
import rc_command
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

    def send_rc_command(self, x_velocity, y_velocity, z_velocity):
        '''
        x_velocity, y_velocity, and z_velocity are percentages of the maximum
        velocity of the quadcopter in the given axis. Must be floats between 0
        and 1
        '''
        command = rc_command.RCCommand({'roll': x_velocity,
                                        'pitch': y_velocity,
                                        'throttle': z_velocity})
        self.quadcopter.send_rc_command(command)

    def rc_square_dance(self):
        rospy.loginfo("forward")
        command = rc_command.RCCommand({'pitch': 0.9})
        self.quadcopter.send_rc_command(command)
        rospy.sleep(2)
        rospy.loginfo("still")
        command = rc_command.RCCommand()
        self.quadcopter.send_rc_command(command)
        rospy.sleep(2)

        # rospy.loginfo("right")
        # command = rc_command.RCCommand({'roll': 0.9})
        # self.quadcopter.send_rc_command(command)
        # rospy.sleep(2)
        # rospy.loginfo("still")
        # command = rc_command.RCCommand()
        # self.quadcopter.send_rc_command(command)
        # rospy.sleep(2)

        # rospy.loginfo("backward")
        # command = rc_command.RCCommand({'pitch': 0.1})
        # self.quadcopter.send_rc_command(command)
        # rospy.sleep(2)
        # rospy.loginfo("still")
        # command = rc_command.RCCommand()
        # self.quadcopter.send_rc_command(command)
        # rospy.sleep(2)

        # rospy.loginfo("left")
        # command = rc_command.RCCommand({'roll': 0.1})
        # self.quadcopter.send_rc_command(command)
        # rospy.sleep(2)
        # rospy.loginfo("still")
        # command = rc_command.RCCommand()
        # self.quadcopter.send_rc_command(command)
        # rospy.sleep(2)

    def rc_land_on_fiducial(self):
        found, _, _ = self.find_landing_site()
        if found:
            dz = self.landing_site.center.position.z
            while dz > 1 and not rospy.is_shutdown():
                dz = self.landing_site.center.position.z
                dx = self.landing_site.center.position.x
                dy = self.landing_site.center.position.y

                self.proportional_position(dx, dy, dz)
                rospy.sleep(0.1)

            for i in range(5):
                rospy.loginfo("Descending")
                self.send_rc_command(0.5, 0.5, 0.25)
                rospy.sleep(1)
        rospy.loginfo("Finished landing")

    def calculate_planar_speed(self, pos):
        max_speed = 0.9
        min_speed = 0.1
        tolerance = 0.5
        return ((max_speed - min_speed) / (1 + numpy.exp(-tolerance * pos)))\
            + min_speed

    def calculate_rate_of_descent(self, dx, dy):
        max_throttle = 0.5
        min_throttle = 0.25
        tolerance = 0.5
        distance = numpy.linalg.norm([dx, dy])
        return max_throttle - \
            min_throttle * numpy.exp(-tolerance * (distance ** 2))

    def proportional_position(self, dx, dy, dz):
        '''
        Receives the relative position of the fiducial and navigates towards it
        using a proportional controller. The rate of descent increases the more
        centered the quadcopter is over the fiducial.
        '''
        x_diff = self.calculate_planar_speed(dx)
        y_diff = self.calculate_planar_speed(dy)
        z_diff = self.calculate_rate_of_descent(dx, dy)
        self.send_rc_command(x_diff, y_diff, z_diff)

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
            self.go_to_waypoints([waypoint], 5)
            found, goal_lat, goal_long = self.find_landing_site(10)
            if found:
                return True, goal_lat, goal_long
        return False, 0, 0

    def find_landing_site(self, wait_seconds=60):
        '''
        Executes a search behavior for the fiducial, return its placement of
        the fiducial it has, in (latitude, longitude) form
        TODO: Make a behavior that takes more data to place the site
        '''
        time_limit = datetime.timedelta(seconds=wait_seconds)
        time_end = datetime.datetime.now() + time_limit
        seen = False
        rospy.loginfo("Searching for landing site, %d seconds..." %
                      (wait_seconds))
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

    def land_on_fiducial_incremental(self, wait_seconds=15):
        '''
        Averages the position of the fiducial, goes to that spot and steps
        down altitude in discrete steps until low enough, then lands
        '''
        found, _, _ = self.find_landing_site(wait_seconds)
        alt = -1.0
        if found:
            goal_lat, goal_long, goal_vertical_dist = \
                self.landing_site.get_average_lat_long(self.quadcopter)
            seen = goal_lat != None
            alt = self.quadcopter.current_rel_alt
            while seen and alt > 2.5:
                if alt > 6.0:
                    next_alt = 5.0
                    wait_seconds = 10
                # elif alt > 3.5:
                else:
                    next_alt = 2
                    wait_seconds = 5
                # else:
                #     next_alt = 1.0
                waypt = {'latitude': goal_lat,
                         'longitude': goal_long,
                         'altitude': next_alt}
                self.go_to_waypoints([waypt], wait_seconds)
                goal_lat, goal_long, goal_vertical_dist = \
                    self.landing_site.get_average_lat_long(self.quadcopter)
                seen = goal_lat != None
                alt = self.quadcopter.current_rel_alt
                seen = goal_lat is not None
        rospy.loginfo("Fiducial found: %s, altitude %f", found, alt)
        self.land()
