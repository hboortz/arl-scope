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

    def hover_in_place(self, extrapolate=False):
        location = self.quadcopter.pose_at(time.time())
        waypoint_data = [{"latitude": location.latitude,
                          "longitude": location.longitude,
                          "altitude": location.altitude}]
        print("Sending hover command...")
        self.go_to_waypoints(waypoint_data)
        print("Hover command sent")

    def check_reached_waypoint(self, waypoint):
        wait_time = 0
        while not self.has_reached_waypoint and wait_time < 50:
            rospy.sleep(5)
            wait_time += 5
            location = self.quadcopter.pose_at(time.time())
            print "--> Traveling to waypoint for %d seconds" % (wait_time)
            print "--> Current position is %s" % (location, )
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
            location = self.quadcopter.pose_at(time.time())
            _, _, dist_from_waypoint = \
                PositionTools.lat_lon_diff(location.latitude,
                                           location.longitude,
                                           wpt_latitude,
                                           wpt_longitude)
            print "Distance to waypoint: " + str(dist_from_waypoint)
            print "Current pos: %s" % (location, )
            return dist_from_waypoint < error_margin
        except AttributeError:  # if haven't gotten current position data
            return False
