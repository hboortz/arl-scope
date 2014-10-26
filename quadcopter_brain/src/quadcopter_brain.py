#!/usr/bin/env python
# 10/22/2014
# Charles O. Goddard

import rospy
import time
#import tf

import roscopter
import roscopter.msg
import roscopter.srv
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu


class QuadcopterBrain(object):
    '''
    High-level quadcopter controller.
    '''
    def __init__(self):
        rospy.Subscriber("filtered_pos",
                         roscopter.msg.FilteredPosition,
                         self.on_position_update)

        self.command_service = rospy.ServiceProxy(
            'command', roscopter.srv.APMCommand
        )
        self.waypoint_service = rospy.ServiceProxy(
            'waypoint', roscopter.srv.SendWaypoint
        )
        # self.land_service = rospy.ServiceProxy(
        #     'land', Empty
        # )

    def fly_path(self, waypoint_data):
        waypoints = [build_waypoint(datum) for datum in waypoint_data]

        # Execute flight plan
        self.command_service(roscopter.srv.APMCommandRequest.CMD_ARM)
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAUNCH)
        print('Launched')
        for waypoint in waypoints:
            self.waypoint_service(waypoint)
            print('Sent waypoint')
            time.sleep(15)
        print('Landing')
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAND)

    def on_position_update(self, data):
        '''
        data: GPS + IMU
        '''
        pass


def build_waypoint(data):
    latitude = data['latitude']
    longitude = data['longitude']
    altitude = data.get('altitude', 8)
    hold_time = data.get('hold_time', 3.0)

    waypoint = roscopter.msg.Waypoint()
    waypoint.latitude = gps_to_mavlink(latitude)
    waypoint.longitude = gps_to_mavlink(longitude)
    waypoint.altitude = int(altitude * 1000)
    waypoint.hold_time = int(hold_time * 1000)  # in ms
    waypoint.waypoint_type = roscopter.msg.Waypoint.TYPE_NAV
    return waypoint


def gps_to_mavlink(coordinate):
    '''
    coordinate: decimal degrees
    '''
    return int(coordinate * 1e+7)


if __name__ == '__main__':
    #rospy.init_node("quadcopter_brain")
    carl = QuadcopterBrain()
    carl.fly_path([
        {'latitude': 42.2926834, 'longitude': -71.2628237},
        {'latitude': 42.2925417, 'longitude': -71.2628411}
    ])

