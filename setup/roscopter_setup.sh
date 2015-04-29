#!/usr/bin/env bash


source file_utils.sh


roscopter_dir() {
	catkin_src
	cd roscopter
}

download_roscopter() {
	if [ ! -d "roscopter" ]
	then
		git clone git@github.com:ElizabethDuncan/roscopter.git
	fi
}

download_submodules() {
	git submodule init
	git submodule update
}

setup_pymavlink() {
	roscopter_dir
	cd mavlink/pymavlink
	sudo python setup.py install
}

setup_roscopter_driver() {
	roscopter_dir
	cd scripts
	chmod 777 driver.py
}

catkin_src
download_roscopter
roscopter_dir
download_submodules
setup_roscopter_driver
setup_pymavlink
catkin_root
catkin_make
