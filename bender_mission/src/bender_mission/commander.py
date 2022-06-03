import numpy as np
import rospy
import actionlib
import tf2_ros
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import PoseStamped 

class Commander:
    def __init__(self):
        self.data = "hello"
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.current_goal = MoveBaseAction(PoseStamped())

    def say(self):
        print(self.data)