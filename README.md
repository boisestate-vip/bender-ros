# bender-ros
This repository contains ROS packages for the Bender robot designed and built by a group of students from the Vertically Integrated Projects (VIP) program at Boise State University.

# Installation

These ROS packages are tested on ROS Noetic. The following commands will:
1) Clone this repo 
2) Install ROS dependencies to build and run the packages in this repository
3) Install Gazebo models
4) Install `udev` rule for Logitech F710 joystick
```
CATKIN_WS=$HOME/projects/bender/ros
mkdir -p $CATKIN_WS/src && cd $CATKIN_WS/src 
git clone https://github.com/boisestate-vip/bender-ros.git
cd $CATKIN_WS
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
cd $CATKIN_WS/src/bender-ros/bender_gazebo/models && ./install_models.sh
cd $CATKIN_WS/src/bender-ros/bender_base/scripts && ./joy_udev.sh
catkin build
```


# Simple Navigation
Launch a simple navigation demo in Gazebo using the following:
```
roslaunch bender_gazebo simple_nav.launch
```
![simple-nav-demo-gif](https://github.com/boisestate-vip/bender-ros/raw/master/media/gifs/first-nav.gif)

