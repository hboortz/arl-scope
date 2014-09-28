#!/usr/bin/env bash

# Install OpenCV
# http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html#linux-installation
# cd
# if [ ! -d "opencv" ]; then
#	git clone https://github.com/Itseez/opencv.git
#	cd ~/opencv
#	mkdir release
#	cd release
#	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
#	make
#	sudo make install
#fi


# Install ARL stuff
cd ~/catkin_ws/src
if [ ! -d "arl-scope" ]; then
	git clone git@github.com:hboortz/arl-scope.git
fi

cd ~/catkin_ws
catkin_make
