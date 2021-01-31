#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include "pid.h"
#include "bender_joints.h"



ros::NodeHandle nh;

PositionJoint pos_joints[4] = {
	PositionJoint(2, 3, 35, 16, 10.0, 0.1, 0.1), // leg_lf_joint
	PositionJoint(4, 5, 36, 17, 10.0, 0.1, 0.1), // leg_rf_joint
	PositionJoint(6, 7, 37, 18, 10.0, 0.1, 0.1), // leg_lh_joint
	PositionJoint(8, 9, 38, 19, 10.0, 0.1, 0.1)  // leg_rh_joint
};
VelocityJoint vel_joints[4] = {
	VelocityJoint(10, 11, 12, 10.0, 0.1, 0.0), // wheel_lf_joint
	VelocityJoint(14, 13, 15, 10.0, 0.1, 0.0), // wheel_rf_joint
	VelocityJoint(29, 28, 27, 10.0, 0.1, 0.0), // wheel_lh_joint
	VelocityJoint(30, 26, 34, 10.0, 0.1, 0.0)  // wheel_rh_joint
};

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
			vel_joints[i].setTarget(cmd_msg.data[i]);
			_jstate_vel[i] = cmd_msg.data[i];
		} else if (4 <= i && i < 8) {
			pos_joints[i].setTarget(cmd_msg.data[i]);
			_jstate_pos[i] = cmd_msg.data[i];
		}
	}
}
ros::Subscriber<std_msgs::Float32MultiArray> cmd_subscriber("cmd_drive", updateCmd);


void setup()
{
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
	for (int i=0; i<4; i++)
	{
		float this_joint_effort;
		pos_joints[i].update(50);
		pos_joints[i].getEffort(this_joint_effort);
		_jstate_eff[i] = this_joint_effort;
		vel_joints[i].update(50);
		vel_joints[i].getEffort(this_joint_effort);
		_jstate_eff[i+4] = this_joint_effort;
	}
	
	for (int i=0; i<8; i++)
	{
		feedback_msg.position[i] = _jstate_pos[i];
		feedback_msg.velocity[i] = _jstate_vel[i];
		feedback_msg.effort[i]   = _jstate_eff[i];
	}
	state_publisher.publish( &feedback_msg );
	nh.spinOnce();
	delay(100);
}
