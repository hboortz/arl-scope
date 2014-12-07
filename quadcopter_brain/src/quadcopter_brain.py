#!/usr/bin/env python

import rospy
import time
import json

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
        time.sleep(5)
        self.trigger_auto_service()
        self.adjust_throttle_service()
        for waypoint in waypoints:
            self.waypoint_service(waypoint)
            #self.trigger_auto_service()
            print('Sent waypoint %d, %d' %(waypoint.latitude, waypoint.longitude))
            time.sleep(15)
            #TODO write a smart function to determine wait time
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAND)
        print('Landing')

    #def reached_waypoint(self, waypoint):
    #    return 

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


def open_waypoint_file(filename):
    f = open(filename)
    waypoints = json.load(f)
    return waypoints 


if __name__ == '__main__':
    #rospy.init_node("quadcopter_brain")
    carl = QuadcopterBrain()
    carl.clear_waypoints_service()
    #great_lawn_waypoints = open_waypoint_file(
    #   "waypoint_data/great_lawn_waypoints.json")
    #carl.fly_path([great_lawn_waypoints['A'], great_lawn_waypoints['B']])
    carl.fly_path([
        {'latitude': 42.2918389, 'longitude': -71.2625737},
        {'latitude': 42.2917346, 'longitude': -71.2624889},
        {'latitude': 42.2918441, 'longitude': -71.2624461}])
