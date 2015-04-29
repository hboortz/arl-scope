#!/usr/bin/env bash


catkin_root() {
	catkin_ws=$(find ~ -type d -name "catkin_ws")
	cd $catkin_ws
}


catkin_src() {
	catkin_root
	cd src
}
