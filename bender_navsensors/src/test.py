import rospy
from numpy import *
from std_msgs.msg import String
from mavros_msgs.msg import *
from mavros_msgs.srv import *
    
def bender_navsensors():
    pub = rospy.Publisher('bender_navsensors', double, queue_size=10)
    rospy.init_node('bender_navsensors', anonymous=False)
    rate = rospy.Rate(60) # 60hz

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
    ##    float32 test_altitude = Altitude()
        pub.publish("dank memes")
        rate.sleep()
   
if __name__ == '__main__':
       try:
          bender_navsensors()
       except rospy.ROSInterruptException:
           pass
