#!/usr/bin/env python

import rospy
import time

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

        self.clear_waypoints_service = rospy.ServiceProxy(
            'clear_waypoints', Empty)
        self.command_service = rospy.ServiceProxy(
            'command', roscopter.srv.APMCommand)
        self.waypoint_service = rospy.ServiceProxy(
            'waypoint', roscopter.srv.SendWaypoint)
        self.trigger_auto_service = rospy.ServiceProxy(
            'trigger_auto', Empty)
        self.adjust_throttle_service = rospy.ServiceProxy(
            'adjust_throttle', Empty)
        # self.land_service = rospy.ServiceProxy(
        #     'land', Empty
        # )
        

    def fly_path(self, waypoint_data):
        waypoints = [build_waypoint(datum) for datum in waypoint_data]

        # Execute flight plan
        self.command_service(roscopter.srv.APMCommandRequest.CMD_ARM)
        print('Armed')
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAUNCH)
        print('Launched')
        self.trigger_auto_service()
        time.sleep(5)
        self.adjust_throttle_service()
        for waypoint in waypoints:
            self.waypoint_service(waypoint)
            print('Sent waypoint %d, %d' %(waypoint.latitude, waypoint.longitude))
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
    carl.clear_waypoints_service()
    carl.fly_path([
        {'latitude': 42.2918389, 'longitude': -71.2625737},
        {'latitude': 42.2917346, 'longitude': -71.2624889},
        {'latitude': 42.2918441, 'longitude': -71.2624461}])

#Upper great lawn
#        {'latitude': 42.2929217, 'longitude': -71.2633305},
#        {'latitude': 42.2931392, 'longitude': -71.2632456}
#    ])

# Lower great lawn
#        {'latitude': 42.2927971, 'longitude' : -71.2630297},
#        {'latitude': 42.2924562, 'longitude': -71.2630885},
#        {'latitude': 42.2928173, 'longitude': -71.2631555}
#    ])
