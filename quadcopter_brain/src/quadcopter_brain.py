#!/usr/bin/env python

import time
import json
import os

import rospkg
import rospy
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


    def send_waypoint(self, waypoint):
        successfully_sent_waypoint = False
        tries = 0
        while not successfully_sent_waypoint and tries < 5:
            res = self.waypoint_service(waypoint)
            tries += 1
            successfully_sent_waypoint = res.result
            if successfully_sent_waypoint:
                print('Sent waypoint %d, %d' % (waypoint.latitude,
                                                waypoint.longitude))
                time.sleep(15)
            else:
                print("Failed to send waypoint %d, %d" % (waypoint.latitude,
                                                          waypoint.longitude))
                time.sleep(0.1)
                if tries == 4:
                    print("Tried 5 times and giving up")
                else:
                    print("Trying again. Tries: %d" % (tries+1))

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
            self.send_waypoint(waypoint)
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAND)
        print('Landing')

    def reached_waypoint(self, waypoint):
        pass

    def on_position_update(self, data):
        '''
        data: GPS + Compass (+ IMU?)
        '''
        self.latitude = data.latitude / 1e7
        self.longitude = data.longitude / 1e7
        self.altitude = data.altitude / 1e3
        self.heading = data.heading / 1e2


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
    return int(coordinate * 1e7)


def open_waypoint_file(filename):
    rospack = rospkg.RosPack()
    quadcopter_brain_path = rospack.get_path("quadcopter_brain")
    source_path = "src"
    file_path = os.path.join(quadcopter_brain_path, source_path, filename)
    with open(file_path, "r") as f:
        waypoints = json.load(f)
    return waypoints


if __name__ == '__main__':
    #rospy.init_node("quadcopter_brain")
    carl = QuadcopterBrain()
    carl.clear_waypoints_service()
    great_lawn_waypoints = open_waypoint_file(
        "waypoint_data/great_lawn_waypoints.json")
    carl.fly_path([great_lawn_waypoints['V1'], great_lawn_waypoints['V2'],
                   great_lawn_waypoints['V3']])
