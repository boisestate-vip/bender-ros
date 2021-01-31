#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include "pid.h"
#include "bender_joints.h"


VelocityJoint hub1(14, 13, 15);
PositionJoint planet1(2, 3, 35, 16);
ros::NodeHandle nh;


char *_jstate_name[] = {
	"wheel_lf_joint", "wheel_rf_joint", "wheel_lh_joint", "wheel_rh_joint",
	"leg_lf_joint", "leg_rf_joint", "leg_lh_joint", "leg_rh_joint"
};
float _jstate_pos[8] = {0,0,0,0,0,0,0,0};
float _jstate_vel[8] = {0,0,0,0,0,0,0,0};
float _jstate_eff[8] = {0,0,0,0,0,0,0,0};
sensor_msgs::JointState feedback_msg;
ros::Publisher state_publisher("feedback", &feedback_msg);


void updateCmd(const std_msgs::Float32MultiArray &cmd_msg)
{
	for (unsigned int i=0; i<cmd_msg.data_length; i++)
	{
		if (i < 4) {
			_jstate_vel[i] = cmd_msg.data[i];
		} else if (4 <= i && i < 8) {
			_jstate_pos[i] = cmd_msg.data[i];
		}
	}
}
ros::Subscriber<std_msgs::Float32MultiArray> cmd_subscriber("cmd_drive", updateCmd);


void setup()
{
	pinMode(22, OUTPUT);
	nh.initNode();
	feedback_msg.name            = _jstate_name;
	feedback_msg.position        = _jstate_pos;
	feedback_msg.velocity        = _jstate_vel;
	feedback_msg.effort          = _jstate_eff;
	feedback_msg.name_length     = 8;
	feedback_msg.position_length = 8;
	feedback_msg.velocity_length = 8;
	feedback_msg.effort_length   = 8;
	nh.advertise(state_publisher);
	nh.subscribe(cmd_subscriber);
}

void loop()
{
	for (unsigned int i=0; i<8; i++)
	{
		if (i < 4) {
			feedback_msg.position[i] = _jstate_pos[i];
		} else if (4 <= i && i < 8) {
			feedback_msg.velocity[i] = _jstate_vel[i];
		}
	}
	state_publisher.publish( &feedback_msg );
	nh.spinOnce();
	delay(500);
}
