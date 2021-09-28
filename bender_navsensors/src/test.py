import rospy
   4 from std_msgs.msg import String
   5 
   6 def talker():
   7     pub = rospy.Publisher('chatter', String, queue_size=10)
   8     rospy.init_node('talker', anonymous=True)
   9     rate = rospy.Rate(10) # 10hz
  10     while not rospy.is_shutdown():
  11         hello_str = "hello world %s" % rospy.get_time()
  12         rospy.loginfo(hello_str)
  13         pub.publish(hello_str)
  14         rate.sleep()
  15 
  16 if __name__ == '__main__':
  17     try:
  18         talker()
  19     except rospy.ROSInterruptException:
  20         pass
