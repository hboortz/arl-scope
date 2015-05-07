#!/usr/bin/env bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update

sudo apt-get install -y python-rosinstall
sudo apt-get install -y ros-indigo-mavlink
sudo apt-get install -y ros-indigo-mavros
sudo apt-get install -y ros-indigo-geodesy

# Libraries necessary for camera usage (including freeglut)
sudo apt-get install -y ros-indigo-uvc-camera
sudo apt-get install -y freeglut3-dev
# Create camera calibration file
sudo touch /opt/ros/indigo/share/uvc_camera/camera_calibration.yaml
sudo chown $USER:$USER /opt/ros/indigo/share/uvc_camera/camera_calibration.yaml

echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
