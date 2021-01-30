/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>

#include "pid.h"
#include "bender_joints.h"

ros::NodeHandle  nh;
Pid pid(10.0, 0.1, 0.1);
VelocityJoint hub1(14, 13, 15);
PositionJoint planet1(2, 3, 35, 16);

void messageCb( const std_msgs::Empty& toggle_msg){
  pid.setGains(100.0, 0.1, 0.1);
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );
std_msgs::Float32MultiArray chatter_msg;
ros::Publisher chatter("chatter", &chatter_msg);

float out[3] = {0.0, 0.0, 0.0};

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  chatter_msg.data_length = 3;
}

void loop()
{
  pid.getGains(out[0],out[1],out[2]);
  chatter_msg.data = out;
  chatter.publish( &chatter_msg );
  nh.spinOnce();
  delay(500);
}
