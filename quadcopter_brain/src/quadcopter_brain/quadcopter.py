#!/usr/bin/env python

import rospy
import roscopter.msg
import roscopter.srv
import std_srvs.srv

from position_tools import PositionTools


class Quadcopter(object):
    def __init__(self):
        rospy.init_node("Quadcopter")
        self._clear_waypoints_service = rospy.ServiceProxy(
            'clear_waypoints', std_srvs.srv.Empty)
        self._command_service = rospy.ServiceProxy(
            'command', roscopter.srv.APMCommand)
        self._waypoint_service = rospy.ServiceProxy(
            'waypoint', roscopter.srv.SendWaypoint)
        self._trigger_auto_service = rospy.ServiceProxy(
            'trigger_auto', std_srvs.srv.Empty)
        self._adjust_throttle_service = rospy.ServiceProxy(
            'adjust_throttle', std_srvs.srv.Empty)

        rospy.Subscriber("/filtered_pos", roscopter.msg.FilteredPosition,
                         self._position_callback)
        rospy.Subscriber("/send_rc", roscopter.msg.RC,
                         self._send_rc_callback)
        self.rc_pub = rospy.Publisher('/send_rc', roscopter.msg.RC,
                                      queue_size=10, latch=True)

        self.current_lat = 0.0
        self.current_long = 0.0
        self.current_rel_alt = 0.0
        self.current_alt = 0.0
        self.heading = 0.0
        # [side tilt, front tilt, throttle, spin, SOMETHING]
        self.rc_cmd = [1800 for i in range(8)]

    def _position_callback(self, data):
        self.current_lat = PositionTools.mavlink_to_gps(data.latitude)
        self.current_long = PositionTools.mavlink_to_gps(data.longitude)
        self.heading = PositionTools.mavlink_to_degrees(data.heading)
        self.current_alt = PositionTools.mavlink_to_altitude(data.altitude)
        self.current_rel_alt =\
            PositionTools.mavlink_to_altitude(data.relative_altitude)

    def _send_rc_callback(self, data):
        pass

    def clear_waypoints(self):
        rospy.loginfo('Sending clear waypoints command...')
        self._clear_waypoints_service()
        rospy.loginfo('Cleared waypoints')

    def arm(self):
        rospy.loginfo('Sending arm command...')
        self._command_service(roscopter.srv.APMCommandRequest.CMD_ARM)
        rospy.loginfo('Armed')

    def return_rc_control(self):
        print('Returning RC Control...')
        self._command_service(roscopter.srv.APMCommandRequest.CMD_SET_LOITER)
        print('RC in control')

    def launch(self, max_num_tries=5):
        rospy.loginfo('Sending launch command...')
        self._send_cmd_and_check_for_success("Launch",
                                    roscopter.srv.APMCommandRequest.CMD_LAUNCH,
                                    max_num_tries=max_num_tries)
        rospy.loginfo('Launched')

    def land(self, max_num_tries=5):
        rospy.loginfo('Sending land command...')
        self._send_cmd_and_check_for_success("Land",
                                    roscopter.srv.APMCommandRequest.CMD_LAND,
                                    max_num_tries=max_num_tries)
        rospy.loginfo('Landing')

    def _send_cmd_and_check_for_success(self, name_of_cmd, cmd_to_send,
                                        max_num_tries):
        successful_cmd_send = False
        tries = 0
        while not successful_cmd_send and tries < max_num_tries:
            res = self._command_service(cmd_to_send)
            successful_cmd_send = res.result
            tries += 1
            self._print_cmd_send_status(name_of_cmd, successful_cmd_send,
                                        tries, max_num_tries)
            rospy.sleep(0.1)
        return successful_cmd_send

    def _print_cmd_send_status(self, name_of_cmd, successful_cmd_send, tries,
                               max_num_tries):
        if successful_cmd_send:
            rospy.loginfo("Successfully %sed", name_of_cmd.lower())
        else:
            rospy.loginfo("%s failed", name_of_cmd)
            if tries == max_num_tries:
                rospy.loginfo("Tried %d times and giving up", tries)
            else:
                rospy.loginfo("Retrying. Tries: %d", tries)

    def send_waypoint(self, waypoint, max_num_tries=5):
        self._set_auto_mode()
        sent_waypoint = False
        tries = 0

        while not sent_waypoint and tries < max_num_tries:
            res = self._waypoint_service(waypoint)
            sent_waypoint = res.result
            tries += 1
            self._print_send_waypoint_status(
                waypoint, sent_waypoint, tries, max_num_tries)
            rospy.sleep(0.1)

        return sent_waypoint

    def send_rc_command(self, rc_command):
        print "sending rc command"
        self.rc_pub.publish(rc_command.to_roscopter())

    def _print_send_waypoint_status(self, waypoint, sent_waypoint,
                                    tries, max_num_tries):
        if sent_waypoint:
            rospy.loginfo('Sent waypoint\n\tlat: %d\n\tlon: %d\n\talt: %d',
                  waypoint.latitude, waypoint.longitude, waypoint.altitude)
        else:
            rospy.loginfo('Failed to send waypt\n\tlat:%d\n\tlon:%d\n\talt:%d',
                  waypoint.latitude, waypoint.longitude, waypoint.altitude)
            if tries == max_num_tries:
                rospy.loginfo("Tried %d times and giving up", tries)
            else:
                rospy.loginfo("Retrying. Tries: %d", tries)

    def _set_auto_mode(self):
        '''
            TODO: Explain why it is necessary to trigger_auto and
            adjust_throttle - b/c ROSCOPTER is dumb
        '''
        self._trigger_auto_service()
        self._adjust_throttle_service()
