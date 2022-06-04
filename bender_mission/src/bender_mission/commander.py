from cmath import isnan
import numpy as np
from math import pi, cos, sin

import rospy
import actionlib
import tf2_ros
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, Float64

class Commander:
    def __init__(self):
        self.data = "hello"
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.current_gps = NavSatFix()
        self.home_heading = np.NaN
        self.home_gps = NavSatFix()
        self.start = rospy.get_param("~start")
        self.goals = rospy.get_param("~goals")
        self.current_goal = PoseStamped()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.compass_subscriber = rospy.Subscriber("compass/heading", Float64, self.compass_callback)
        self.current_gps_subscriber = rospy.Subscriber("gps/fix", NavSatFix, self.gps_callback)
        
    def gps_callback(self, msg):
        self.current_gps = msg

    def compass_callback(self, msg):
        if isnan(self.home_heading):
            self.home_heading = msg.data*pi/180.0

    def action_callback(self, fbk):
        return None
        
    def execute(self):
        if isnan(self.home_heading):
            rospy.logwarn_throttle(1.0, "Waiting to receive initial heading..")
            return
        for goal in self.goals:
            delta_lat  = goal[0] - self.start['lat'] # self.home_gps.latitude
            delta_long = goal[1] - self.start['long'] # self.home_gps.longitude
            desired_point = self.latlong_to_meters(self.current_gps.latitude, delta_lat, delta_long)
            desired_point[1] = -desired_point[1] 
            self.current_goal.pose.position.x = desired_point[0]
            self.current_goal.pose.position.y = desired_point[1]
            self.current_goal.pose.orientation = Quaternion(*quaternion_from_euler(0,0,self.home_heading))
            self.current_goal.header = Header()
            self.current_goal.header.stamp = rospy.Time.now()
            self.current_goal.header.frame_id = "map"
            print(desired_point)
            
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose = self.current_goal
            self.action_client.send_goal(goal=move_base_goal, feedback_cb=self.action_callback)
            self.action_client.wait_for_result()

    def latlong_to_meters(self, lat_current, lat_delta, long_delta):
        n = lat_delta * 111321.5
        e = long_delta * cos(lat_current*pi/180.0) * 111321.5
        print([n,e])
        R = np.array([
            [ cos(-self.home_heading), -sin(-self.home_heading)],
            [ sin(-self.home_heading),  cos(-self.home_heading)]
        ])
        return np.dot(R, np.array([n,e]))
        # hdg = self.home_heading
