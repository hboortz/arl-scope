import time
import rospy
import roscopter.msg
import roscopter.srv
import std_srvs.srv

from position_tools import PositionTools, QuadcopterPose
from latency_tools import TemporalBuffer


class Quadcopter(object):
    def __init__(self, telemetry_delay=0.3):
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

        self.pose_buffer = TemporalBuffer()
        self.telemetry_delay = telemetry_delay
        rospy.Subscriber("/filtered_pos", roscopter.msg.FilteredPosition,
                         self._position_callback)

    def last_known_pose(self):
        return self.pose_buffer.last()

    def pose_at(self, timestamp):
        return self.pose_buffer.query(timestamp)

    def _position_callback(self, data):
        t = time.time()
        pose = QuadcopterPose.from_mavlink(data)
        self.pose_buffer.add(pose, t)

    def arm(self):
        print('Sending arm command...')
        self._command_service(roscopter.srv.APMCommandRequest.CMD_ARM)
        print('Armed')

    def launch(self, max_num_tries=5):
        print('Sending launch command...')
        successful_launch = False
        tries = 0
        while not successful_launch and tries < max_num_tries:
            res = self._command_service(
                roscopter.srv.APMCommandRequest.CMD_LAUNCH)
            successful_launch = res.result
            tries += 1
            self._print_launch_status(successful_launch, tries, max_num_tries)
            rospy.sleep(0.1)
        return successful_launch

    def _print_launch_status(self, successful_launch, tries, max_num_tries):
        if successful_launch:
            print("Successfully launched")
        else:
            print("Launch failed")
            if tries == max_num_tries:
                print("Tried %d times and giving up" % (tries))
            else:
                print("Retrying. Tries: %d" % (tries))

    def land(self):
        print('Sending land command...')
        self._command_service(roscopter.srv.APMCommandRequest.CMD_LAND)
        print('Landing')

    def clear_waypoints(self):
        print('Sending clear waypoints command...')
        self._clear_waypoints_service()
        print('Cleared waypoints')

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

    def _set_auto_mode(self):
        '''
            TODO: Explain why it is necessary to trigger_auto and
            adjust_throttle - b/c ROSCOPTER is dumb
        '''
        self._trigger_auto_service()
        self._adjust_throttle_service()

    def _print_send_waypoint_status(self, waypoint, sent_waypoint,
                                    tries, max_num_tries):
        if sent_waypoint:
            print('Sent waypoint %d, %d' % (waypoint.latitude,
                                            waypoint.longitude))
        else:
            print("Failed to send waypoint %d, %d" % (waypoint.latitude,
                                                      waypoint.longitude))
            if tries == max_num_tries:
                print("Tried %d times and giving up" % (tries))
            else:
                print("Retrying. Tries: %d" % (tries))
