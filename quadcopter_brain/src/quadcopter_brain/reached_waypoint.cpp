#include <cstdio>

#include <ros/ros.h>
#include <roscopter/Waypoint.h> // package/(name of .msg file)
#include <roscopter/FilteredPosition.h>
#include <ros/console.h>
// #include <roscopter_msg/Waypoint.h>


/*  
    bool check_reached_waypoint
def check_reached_waypoint(self, waypoint, max_wait_time=60, wait_time=0):
    
    bool has_reached_waypoint
def has_reached_waypoint(self, waypoint, xy_error_margin=3,
                             alt_error_margin=1):
    void waypoint_timeout_choice 
def waypoint_timeout_choice(self, waypoint, curr_wait_time):
*/


// streams vs stdio
// oop?
// waypoint struct?
// default arg vs overloading


struct Location {
  float lat;
  float lon;
  float alt;
  float rel_alt;
  // float heading;
};


class WaypointCheck {
	public:
    WaypointCheck();
    bool checkReachedWaypoint(Location &waypoint, Location &current);
        // check if within radius (lat/long), and check alt
  	void posCallback(const roscopter::FilteredPosition &pos);
  	Location wayptLoc;
  	Location currLoc;
};


WaypointCheck::WaypointCheck() {
}


void WaypointCheck::posCallback(const roscopter::FilteredPosition &pos)
   {
    printf("%d", pos.relative_altitude);
    currLoc.lat = pos.latitude;
    ROS_INFO_STREAM("Here");
        // ROS_INFO("I heard: [%s]", pos->data.c_str());
    }

// void messageCallback(const quadcopter_brian::node_example_data::ConstPtr &msg)

int main(int argc, char **argv) {
	  ros::init(argc, argv, "listener");
		WaypointCheck currWaypt = WaypointCheck();
		ros::NodeHandle nh;
  	ros::Subscriber posSub = nh.subscribe("/filtered_pos", 10, &WaypointCheck::posCallback, &currWaypt);
  	printf("curr lat %f", currWaypt.currLoc.lat);
    ros::spin();
    ros::shutdown();
    return 0;
}

// [WARN] [WallTime: 1428516516.448570] Could not process inbound connection: topic types do not match: [roscopter/Waypoint] vs. [roscopter/FilteredPosition]{'topic': '/filtered_pos', 'tcp_nodelay': '0', 'md5sum': '6dac49f8cf4308a151d9b97deaca8a40', 'type': 'roscopter/Waypoint', 'callerid': '/listener'}

// [WARN] [WallTime: 1428516569.338363] Could not process inbound connection: topic types do not match: [roscopter/Waypoint] vs. [roscopter/FilteredPosition]{'topic': '/filtered_pos', 'tcp_nodelay': '0', 'md5sum': '6dac49f8cf4308a151d9b97deaca8a40', 'type': 'roscopter/Waypoint', 'callerid': '/listener'}


