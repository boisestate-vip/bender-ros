# bender-ros
This repository contain ROS code for the Bender robot designed and built by a group of students from the Vertically Integrated Projects (VIP) program at Boise State University.

# Installation
Put the contents of this repository into the `<your_catkin_ws>/src` then run `catkin_make`. After it is done building, run
```
rosdep install bender_robot
```
to install the required ROS packages.

# Simple Navigation
Launch a simple navigation demo in Gazebo using the following:
```
roslaunch bender_gazebo simple_nav.launch
```
![Alt Text](https://github.com/boisestate-vip/bender-ros/raw/master/media/gifs/first-nav.gif)

