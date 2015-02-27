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

    def go_to_waypoints(self, waypoint_data):
        waypoints = [
            WaypointTools.build_waypoint(datum) for datum in waypoint_data]
        for waypoint in waypoints:
            if self.quadcopter.send_waypoint(waypoint):
                self.print_position()                               # Remove after testing hover
                print "Waiting 5 seconds until hover..."            # Remove after testing hover
                rospy.sleep(5.0)                                    # Remove after testing hover
                self.print_position()                               # Remove after testing hover
                print "Hovering"                                    # Remove after testing hover
                self.hover_in_place()                               # Remove after testing hover
                print "Waiting for 15 seconds until resuming..."    # Remove after testing hover
                rospy.sleep(12.0)                                   # Remove after testing hover
                self.print_position()                               # Remove after testing hover
                print "3 seconds until resuming..."                 # Remove after testing hover
                rospy.sleep(3.0)                                    # Remove after testing hover
                self.print_position()                               # Remove after testing hover
                print "Resuming"                                    # Remove after testing hover
                # print("Waypoint sent, sleeping 15 seconds for arrival")   # Uncomment after testing hover
                # rospy.sleep(15)                                           # Uncomment after testing hover
                # print("15 seconds passed, moving on")                     # Uncomment after testing hover
                # # self.check_reached_waypoint(waypoint)                   # Uncomment after testing hover

# Remove this after testing hover
    def print_position(self):
        print("Printing position -------------------------------------")
        print("Latitude: %.8f" % self.quadcopter.current_lat)
        print("Longitude: %.8f" % self.quadcopter.current_long)
        print("Altitude: %.2f" % self.quadcopter.current_rel_alt)
        print "Position in m: ", PositionTools.lat_lon_to_meters([[self.quadcopter.current_lat, self.quadcopter.current_long]])


    def fly_path(self, waypoint_data):
        self.quadcopter.launch()
        self.go_to_waypoints(waypoint_data)
        self.quadcopter.land()

    def hover_in_place(self):
        print "AM I HOVERING?"
        waypoint_data = [{"latitude": self.quadcopter.current_lat,
                          "longitude": self.quadcopter.current_long,
                          "altitude": self.quadcopter.current_rel_alt}]
        waypoints = [WaypointTools.build_waypoint(datum) for datum in waypoint_data]
        for waypoint in waypoints:
            print waypoint
            self.quadcopter.send_waypoint(waypoint)

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
