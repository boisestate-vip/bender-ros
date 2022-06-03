#!/usr/bin/env python

import rospy
from bender_mission.commander import Commander

if __name__ == '__main__':
    rospy.init_node('bender_mission')
    cmd = Commander()
    cmd.action_client.wait_for_server()
    rate = rospy.Rate(10);
    while not rospy.is_shutdown():
        cmd.execute()
        rate.sleep()
    