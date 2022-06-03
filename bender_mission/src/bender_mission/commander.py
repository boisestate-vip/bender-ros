import numpy as np
from math import pi, cos, sin

import rospy
import actionlib
import tf2_ros

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class Commander:
    def __init__(self):
        self.data = "hello"
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.current_position = Pose()
        self.current_gps = NavSatFix()
        self.start = rospy.get_param("~start")
        self.goals = rospy.get_param("~goals")
        self.current_goal = Pose()
        self.current_goal_action = MoveBaseAction()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.current_position_subscriber = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.current_gps_subscriber = rospy.Subscriber("gps/fix", NavSatFix, self.gps_callback)

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose
        
    def gps_callback(self, msg):
        self.current_gps = msg
        
    def execute(self): 
        print(self.start['x'])
        print(self.start['y'])
        # for goal in self.goals:
        #     current_goal_action.target_pose.pose.position.x = goal[0]
        #     current_goal_action.target_pose.pose.position.y = goal[1]
        #     action_client.send_goal(current_goal_action)

def latlong_to_meters(lat_current, lat_delta, long_delta)
    n = lat_delta * 111321.5
    e = long_delta * cos(lat_current) * 111321.5
    return n, e