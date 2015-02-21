import rospy
# TODO: Determine if this import is needed
# import roscopter
import roscopter.msg
import roscopter.srv
# TODO: Determine if this import is correctly used
import std_srvs.srv


class Quadcopter(object):
    def __init__(self):
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

        self.current_lat = 0.0
        self.current_long = 0.0
        self.current_rel_alt = 0.0
        self.current_alt = 0.0
        self.heading = 0.0
        rospy.Subscriber("/filtered_pos", roscopter.msg.FilteredPosition,
                         self._position_callback)

    def _position_callback(self, data):
        self.current_lat = data.latitude
        self.current_long = data.longitude
        self.current_rel_alt = data.relative_altitude
        self.current_alt = data.altitude
        self.heading = data.heading

    def arm(self):
        self._command_service(roscopter.srv.APMCommandRequest.CMD_ARM)
        print('Armed')

    def launch(self, system_wait=5):
        self._command_service(roscopter.srv.APMCommandRequest.CMD_LAUNCH)
        print('Launched, waiting for %d seconds' % (system_wait))
        rospy.sleep(system_wait)

    def land(self):
        self._command_service(roscopter.srv.APMCommandRequest.CMD_LAND)
        print('Landing')

    def clear_waypoints(self):
        self._clear_waypoints_service()

    def send_waypoint(self, waypoint, max_num_tries=5):
        self._set_auto_mode()
        successfully_sent_waypoint = False
        tries = 0

        while not successfully_sent_waypoint and tries < max_num_tries:
            res = self._waypoint_service(waypoint)
            successfully_sent_waypoint = res.result
            tries += 1
            self._report_waypoint_status(
                successfully_sent_waypoint, tries, max_num_tries)
            if successfully_sent_waypoint:
                print('Sent waypoint %d, %d' % (waypoint.latitude,
                                                waypoint.longitude))
                print self.check_reached_waypoint(waypoint)
            else:
                print("Failed to send waypoint %d, %d" % (waypoint.latitude,
                                                          waypoint.longitude))
                rospy.sleep(0.1)
                if tries == max_num_tries:
                    print("Tried %d times and giving up" % (tries))
                else:
                    print("Retrying. Tries: %d" % (tries))

        return successfully_sent_waypoint

    def _set_auto_mode(self):
        '''
            TODO: Explain why it is necessary to trigger_auto and
            adjust_throttle - b/c ROSCOPTER is dumb
        '''
        self._trigger_auto_service()
        self._adjust_throttle_service()
