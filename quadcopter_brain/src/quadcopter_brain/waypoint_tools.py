import os
import json

import rospkg
import roscopter.msg

from position_tools import PositionTools


class WaypointTools():
    @staticmethod
    def build_waypoint(data):
        '''
        data: dictionary with latitude and longitude
              (altitude and hold_time optional)
        '''
        latitude = data['latitude']
        longitude = data['longitude']
        altitude = data.get('altitude', 8)
        hold_time = data.get('hold_time', 3.0)

        waypoint = roscopter.msg.Waypoint()
        waypoint.latitude = PositionTools.gps_to_mavlink(latitude)
        waypoint.longitude = PositionTools.gps_to_mavlink(longitude)
        waypoint.altitude = PositionTools.altitude_to_mavlink(altitude)
        waypoint.hold_time = int(hold_time * 1000)  # in ms
        waypoint.waypoint_type = roscopter.msg.Waypoint.TYPE_NAV
        return waypoint

    @staticmethod
    def open_waypoint_file(filename):
        rospack = rospkg.RosPack()
        quadcopter_brain_path = rospack.get_path("quadcopter_brain")
        source_path = "src/waypoint_data"
        file_path = os.path.join(quadcopter_brain_path, source_path, filename)
        with open(file_path, "r") as f:
            waypoints = json.load(f)
        return waypoints
