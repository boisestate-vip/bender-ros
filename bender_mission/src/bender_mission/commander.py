import numpy as np
import rospy
import actionlib
import tf2_ros

class Commander:
    def __init__(self):
        self.data = "hello"

    def say(self):
        print(self.data)