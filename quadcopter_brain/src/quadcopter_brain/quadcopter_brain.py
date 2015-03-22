import rospy

from position_tools import PositionTools
from waypoint_tools import WaypointTools
import quadcopter


class QuadcopterBrain(object):
    '''
    High-level quadcopter controller.
    '''
    def __init__(self):
        self.quadcopter = quadcopter.Quadcopter()

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
