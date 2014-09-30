#!/usr/bin/env bash

# Install ARL stuff
cd ~/catkin_ws/src
if [ ! -d "arl-scope" ]; then
	git clone git@github.com:hboortz/arl-scope.git
fi

cd ~/catkin_ws
catkin_make
