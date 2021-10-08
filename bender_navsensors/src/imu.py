import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu

pub = rospy.Publisher('bender_navsensors_imu', Float64MultiArray, queue_size=10)
msg = Float64MultiArray

def callback(data):
    global msg

    rospy.loginfo(rospy.get_caller_id() + "\nAccel x: [{}]\n Accel y: [{}]\n Accel z: [{}]".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    rospy.loginfo(rospy.get_caller_id() + "\nAngular x: [{}]\n Angular y: [{}]\n Angular z: [{}]".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    
    msg.data = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z, data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]

def imu():
    global pub
    
    rospy.init_node('bender_navsensors_imu', anonymous=False)

    rospy.Subscriber("/mavros/imu/data", Imu, callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
   
if __name__ == '__main__':
       try:
          imu()
       except rospy.ROSInterruptException:
           pass
