#!/usr/bin/env python

import datetime
import rospy

import quadcopter_brain
from quadcopter_brain import QuadcopterBrain, build_waypoint
from landing_site import LandingSite


class QuadcopterFiducialBrain(QuadcopterBrain):
    def __init__(self):
        super(QuadcopterFiducialBrain, self).__init__()
        self.landing_site = LandingSite()

    def land_on_fiducial(self):
        time_limit = datetime.timedelta(minutes=1)
        time_end = datetime.datetime.now() + time_limit
        seen = False
        print "Searching for landing site..."
        while datetime.datetime.now() < time_end:
            if self.landing_site.in_view:
                seen = True
                break
            rospy.sleep(0.1)

        print("Found landing site? ", seen)

        if seen:
            print "Landing site info: ", self.landing_site.center
            goal_lat, goal_lon = self.landing_site.latlon(self)
            waypoint = build_waypoint({'latitude': goal_lat,
                                       'longitude': goal_lon,
                                       'altitude': 1.0})

            print "Given waypoint: ", waypoint
            print "Sending waypoint!"
            self.send_waypoint(waypoint)

        print "Landing!!!"
        self.command_service(roscopter.srv.APMCommandRequest.CMD_LAND)

    def setup_test(self, waypoint_data):
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

    # TODO: Add an aqcuire_landing_site() function that hovers for a half
    # second or so and can continuously see the fiducial


if __name__ == '__main__':
    carl = QuadcopterFiducialBrain()
    carl.clear_waypoints_service()
    rospy.init_node("fiducial_landing")
    rospy.sleep(2)
    great_lawn_waypoints = open_waypoint_file(
        "waypoint_data/great_lawn_waypoints.json")
    carl.setup_test([great_lawn_waypoints['A']])
    carl.land_on_fiducial()
