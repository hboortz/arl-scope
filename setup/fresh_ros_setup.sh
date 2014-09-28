#!/usr/bin/env bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update

sudo apt-get install -y python-rosinstall
sudo apt-get install -y ros-indigo-mavros
source ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make

echo "SET UP SSH KEYS BEFORE RUNNING fresh_post_ros_setup.sh"
