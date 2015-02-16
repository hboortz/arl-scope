#!/usr/bin/env python

from copy import deepcopy
import datetime
import os
import time
import json

import rospy

import rospkg
import roscopter
import quadcopter_brain
from quadcopter_brain import QuadcopterBrain, build_waypoint
from landing_site import LandingSite


class QuadcopterFiducialBrain(QuadcopterBrain):
    def __init__(self):
        super(QuadcopterFiducialBrain, self).__init__()
        self.landing_site = LandingSite()

    def land_on_fiducial(self):
        seen, goal_lat, goal_lon = self.find_landing_site()
        if seen:
            waypoint = build_waypoint({'latitude': goal_lat,
                                       'longitude': goal_lon,
                                       'altitude': 1.0})
            print "Given waypoint: ", waypoint
            print "Sending waypoint!"
            self.send_waypoint(waypoint)
        print "Landing!!!"
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAND)

    def find_landing_site(self):
        '''
        Executes a search behavior for the fiducial, return its placement of
        the fiducial it has, in lat, lon form
        TODO: Make a behavior that takes more data to place the site
        '''
        time_limit = datetime.timedelta(minutes=1)
        time_end = datetime.datetime.now() + time_limit
        seen = False
        print "Searching for landing site..."
        while not seen and datetime.datetime.now() < time_end:
            site = deepcopy(self.landing_site)
            seen = site.in_view
            rospy.sleep(0.1)
        if seen:
            print "Landing site info: ", site.center
            return True, site.latlon(self)
        else:
            print "Landing site was NOT FOUND"
            return False, 0, 0

    def dry_run(self, waypoint_data):
        '''
        This test will be redone once quadcopter_brain is broken into more
        configurable methods
        '''
        waypoints = [build_waypoint(datum) for datum in waypoint_data]
        # Execute flight plan
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAUNCH)
        print('Launched')
        time.sleep(5)
        self.trigger_auto_service()
        self.adjust_throttle_service()              
        for waypoint in waypoints:
            self.send_waypoint(waypoint)

def open_waypoint_file(filename):
    rospack = rospkg.RosPack()
    quadcopter_brain_path = rospack.get_path("quadcopter_brain")
    source_path = "src"
    file_path = os.path.join(quadcopter_brain_path, source_path, filename)
    with open(file_path, "r") as f:
        waypoints = json.load(f)
    return waypoints


def main():
    outside = rospy.get_param("outside", False)
    carl = QuadcopterFiducialBrain()
    carl.clear_waypoints_service()
    rospy.init_node("fiducial_landing")
    rospy.sleep(2)
    # great_lawn_waypoints = open_waypoint_file(
        # "waypoint_data/great_lawn_waypoints.json")
    oval_waypoints = open_waypoint_file(
            "waypoint_data/oval_waypoints.json")
    if outside:
        carl.arm()
    carl.dry_run([oval_waypoints['A']])
    carl.land_on_fiducial()


if __name__ == '__main__':
    main()
