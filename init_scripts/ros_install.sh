#!/bin/bash
echo "Beginning ROS Installation"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
echo "Beginning ros-indigo-desktop-full installation..."
sudo apt-get --yes --force-yes install ros-indigo-desktop-full
echo "Setting up rosdep"
sudo rosdep init
rosdep update
echo "Setting up environment"
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "Setting up rosinstall"
sudo apt-get --yes --force-yes install python-rosinstall
echo "Installing dependencies for AR.Drone"
sudo apt-get --yes --force-yes install ros-indigo-gazebo-ros ros-indigo-gazebo-ros-control ros-indigo-camera-info-
manager ros-indigo-ardrone-autonomy libsdl1.2-dev libsdl1.2-dev libudev-dev libiw-dev
sudo apt-get --yes --force-yes ros-indigo-hector-quadrotor ros-indigo-gazebo-ros-control ros-indigo-geographic-msgs
ros-indigo-hector-gazebo ros-indigo-hector-sensors-gazebo ros-indigo-hector-xacro-tools
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
source ~/.bashrc