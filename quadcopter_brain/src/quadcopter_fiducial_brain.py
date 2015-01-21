
#!/usr/bin/env python

import datetime
import rospy

import quadcopter_brain
from quadcopter_brain import QuadcopterBrain, build_waypoint
from landing_site import LandingSite


class QuadcopterFiducialBrain(QuadcopterBrain):
    def __init__(self):
        super(QuadcopterFiducialBrain, self).__init__()
        landing_site = LandingSite()

    def land_on_fiducial(self):
        time_limit = datetime.timedelta(minutes=1)
        time_end = datetime.datetime.Now() + time_limit
        seen = False
        while datetime.datetime.Now() < time_end:
            if landing_site.in_view():
                seen = True
                break
            rospy.sleep(0.1)

        if seen:
            goal_lat, goal_lon, goal_alt = landing_site.latlon(self)
            waypoint = build_waypoint({'latitude': goal_lat,
                                       'longitude': goal_lon,
                                       'altitude': 1.0})
            self.send_waypoint(waypoint)
            rospy.sleep(15.0)
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
    great_lawn_waypoints = open_waypoint_file(
        "waypoint_data/great_lawn_waypoints.json")
    carl.fly_path([great_lawn_waypoints['A']])
    carl.land_on_fiducial()
