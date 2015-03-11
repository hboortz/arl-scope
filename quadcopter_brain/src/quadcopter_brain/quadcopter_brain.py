# Suggested filename change - base_station.py or mission_controller.py

from copy import deepcopy
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
                print("Waypoint sent, sleeping %s seconds for arrival"
                      % time_to_sleep)
                rospy.sleep(time_to_sleep)
                print("%s seconds passed, moving on" % time_to_sleep)
                # self.check_reached_waypoint(waypoint)

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
        print("Sending hover command...")
        self.go_to_waypoints(waypoint_data)
        print("Hover command sent")

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
        wpt_latitude = PositionTools.mavlink_to_gps(waypoint.latitude)
        wpt_longitude = PositionTools.mavlink_to_gps(waypoint.longitude)

        error_margin = 3  # in meters
        print "Checking reached:"
        try:
            _, _, dist_from_waypoint = \
                PositionTools.lat_lon_diff(self.current_lat,
                                           self.current_long,
                                           wpt_latitude,
                                           wpt_longitude)
            print "Distance to waypoint: " + str(dist_from_waypoint)
            print "Current pos: %s, %s" % (self.current_lat, self.current_long)
            return dist_from_waypoint < error_margin
        except AttributeError:  # if haven't gotten current position data
            return False

    def find_landing_site(self, m_or_lat_lon_preference="lat_lon"):
        '''
        Executes a search behavior for the fiducial, return its placement of
        the fiducial it has, in tuple lat, lon form
        TODO: Make a behavior that takes more data to place the site
        '''
        time_limit = datetime.timedelta(minutes=1)
        time_end = datetime.datetime.now() + time_limit
        seen = False
        print "Searching for landing site..."
        while not seen and datetime.datetime.now() < time_end:
            site = deepcopy(self.landing_site)
            seen = site.in_view
            rospy.sleep(0.1)
        if seen:
            print "Landing site FOUND: ", site.center
            return (True,) + site.lat_long(self.quadcopter)
        else:
            print "Landing site was NOT FOUND"
            return False, 0, 0

    def land_on_fiducial_simple(self):
        found, goal_lat, goal_long = self.find_landing_site()
        if found:
            waypt = {'latitude': goal_lat,
                     'longitude': goal_long,
                     'altitude': 1.0}
            self.go_to_waypoints([waypt], 5.0)
        self.land()

    def land_on_fiducial_incremental(self):
        found, _, _ = self.find_landing_site()
        if found:
            alt = self.quadcopter.current_rel_alt
            while alt > 2.0:
                goal_lat, goal_long = \
                    self.landing_site.get_average_lat_long(self.quadcopter)
                waypt = {'latitude': goal_lat,
                         'longitude': goal_long,
                         'altitude': alt - 1.0}
                self.go_to_waypoints([waypt], 5.0)
                alt = self.quadcopter.current_rel_alt
        self.land()

