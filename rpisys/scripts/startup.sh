#!/bin/bash 

if [[ -z "${ROS_INTERFACE}" ]]; then
  netiface=auto
  upattern='state UP'
  wlan0_active=$(ip link show wlan0 | grep -o "$upattern")
  eth0_active=$(ip link show eth0 | grep -o "$upattern")
  if [[ "$upattern" == "${eth0_active}" ]]; then
     netiface=eth0
  elif [[ "$upattern" == "${wlan0_active}" ]]; then
     netiface=wlan0
  else
     netiface=auto
  fi
  export ROS_INTERFACE=$netiface
fi
source /home/ubuntu/ros/primary/devel/setup.bash

logdir=/tmp/ros-log
networklog=$logdir/network.log
mkdir -p $logdir
cat <<EOF > $networklog
=================================================
 ROS NETWORK ENVIRONMENT VARIABLES
=================================================
ROS_MASTER_URI=$ROS_MASTER_URI
ROS_INTERFACE=$ROS_INTERFACE
ROS_IP=$ROS_IP
ROS_HOSTNAME=$ROS_HOSTNAME
=================================================
EOF

cat $networklog

roslaunch bender_base base.launch teleop:=joy

