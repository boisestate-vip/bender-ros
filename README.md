# bender-ros
This repository contain ROS code for the Bender robot designed and built by a group of students from the Vertically Integrated Projects (VIP) program at Boise State University.

# Installation
Put the contents of this repository into your catkin workspace, i.e.
`<your_catkin_ws>/src`, Then, install the required ROS packages to build & run this by executing
```
rosdep install --from-paths src
```
Then, install IGVC Gazebo model by
```
cd bender_gazebo/models
./install_models.sh
```
Finally, build the packages using
```
catkin_make
```

# Simple Navigation
Launch a simple navigation demo in Gazebo using the following:
```
roslaunch bender_gazebo simple_nav.launch
```
![simple-nav-demo-gif](https://github.com/boisestate-vip/bender-ros/raw/master/media/gifs/first-nav.gif)

