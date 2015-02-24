#Olin College Senior Capstone Project - Autonomous Quadcopter Control

#####Done in conjunction with Army Research Labs for the 2014-2015 Olin SCOPE project.
*ARL Liasons*: Brendan Byrne, Jason Gregory

*Olin Team*: Heather Boortz, Elizabeth Duncan, Kaitlin Gallagher, Kyle McConnaughay, Eric Schneider, Charles Goddard

*Project Goal*: Create a code base to autonomously control a quadcopter using basic commands such as arming, launching, navigating to specified gps waypoints, and landing. Use this code base along with image processing to land on both stationary and moving fiducials.

*Current Progress*: Reliable autonomous control and a prototype for landing on a stationary fiducial.

### Running the code
####Setup your environment
Our code has only been tested on Ubuntu 14.04

`setup/setup.sh` is a shell script that should install the necessary software. It can be run by downloading it, make it executable with `chmod u+x`, and then running it with `./setup.sh`. If you find any issues after using this, please file an issue.

####Roscopter
Roscopter creates the services that we publish to in order to send commands across mavlink to the quadcopter. It can be started with 
```roslaunch roscopter.launch```

####Quadcopter Brain
In order to specify custom gps waypoints, create a json file in `arl-scope/quadcopter_brain/src/waypoint_data`, and then modify the arguments to `fly_path` in the main function of `arl-scope/quadcopter_brain/src/quadcopter_brain/quadcopter_brain.py`. The `arm`, `launch`, `go_to_waypoints`, and `land` methods can also be called individually if you only want to use some of them.

In order to run the code in quadcopter brain, run
```rosrun quadcopter_brain quadcopter_brain.py _outside:=True```
If you wish to run the code without arming for testing purposes, simply omit the _outside parameter or set it to `False` instead of `True`.

Thus far, we have only tested our code on the Iris and Iris+ quadcopters made by 3DR, but it should be cross compatible with any multirotors using Mavlink radios and a Pixhawk flight controller.
