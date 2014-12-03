#!/usr/bin/env bash

# Create camera calibration file
sudo touch /opt/ros/indigo/share/uvc_camera/camera_calibration.yaml
sudo chown $USER:$USER /opt/ros/indigo/share/uvc_camera/camera_calibration.yaml

# Install ARL stuff
cd ~/catkin_ws/src
if [ ! -d "arl-scope" ]; then
	git clone git@github.com:hboortz/arl-scope.git
fi

# Get ar_pose
cd ~/catkin_ws/src
if [ ! -d "ar_tools" ]; then
	git clone git@github.com:arlolinscope/ar_tools.git
fi
cd ~/catkin_ws/src/ar_tools
rosmake

cd ~/catkin_ws
catkin_make
