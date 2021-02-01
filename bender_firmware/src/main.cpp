#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include "pid.h"
#include "bender_joints.h"

#define PID_UPDATE_PERIOD_MS 1
#define ROS_PUBLISH_PERIOD_MS 5
#define CMD_RECEIVE_TIMEOUT_MS 100


ros::NodeHandle nh;
PositionJoint pos_joints[4] = {
	PositionJoint(2, 3, 35, 16, 10.0, 0.0, 0.0), // leg_lf_joint
	PositionJoint(4, 5, 36, 17, 10.0, 0.0, 0.0), // leg_rf_joint
	PositionJoint(6, 7, 37, 18, 10.0, 0.0, 0.0), // leg_lh_joint
	PositionJoint(8, 9, 38, 19, 10.0, 0.0, 0.0)  // leg_rh_joint
};
VelocityJoint vel_joints[4] = {
	VelocityJoint(10, 11, 12, 10.0, 0.0, 0.0), // wheel_lf_joint
	VelocityJoint(14, 13, 15, 10.0, 0.0, 0.0), // wheel_rf_joint
	VelocityJoint(29, 28, 27, 10.0, 0.0, 0.0), // wheel_lh_joint
	VelocityJoint(30, 26, 34, 10.0, 0.0, 0.0)  // wheel_rh_joint
};
elapsedMillis since_last_receipt_ms;
elapsedMillis since_last_update_ms;
elapsedMillis since_last_spin_ms;


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
	since_last_receipt_ms = 0;
	for (unsigned int i=0; i<8; i++)
	{
		float target = cmd_msg.data[i];
		if (i < 4) {
			vel_joints[i].setTarget(target);
			_jstate_vel[i] = target;
		} else if (4 <= i && i < 8) {
			pos_joints[i-4].setTarget(target);
			_jstate_pos[i] = target;
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

	since_last_receipt_ms = 0;
	since_last_update_ms = 0;
	since_last_spin_ms = 0;

	/** Enable the motors **/
	// for (int i=0; i<4; i++)
	// {
	// 	vel_joints[i].enable();
	// 	pos_joints[i].enable();
	// }
}


void loop()
{
	/** If it has been too long since last command was received, stop the motors **/
	if (since_last_receipt_ms >= CMD_RECEIVE_TIMEOUT_MS)
	{
		for (int i=0; i<4; i++)
		{
			vel_joints[i].stop();
			pos_joints[i].stop();
		}
	}	
	/** Update the motor commands at a specifeid rate **/
	else if (since_last_update_ms >= PID_UPDATE_PERIOD_MS) 
	{
		for (int i=0; i<4; i++)
		{
			vel_joints[i].update(since_last_update_ms);
			vel_joints[i].getEffort(_jstate_eff[i]);
			vel_joints[i].actuate();
			pos_joints[i].update(since_last_update_ms);
			pos_joints[i].getEffort(_jstate_eff[i+4]);
			pos_joints[i].actuate();
		}
		since_last_update_ms = since_last_update_ms - PID_UPDATE_PERIOD_MS;
	}

	/** Publish via rosserial at a specified rate **/
	if (since_last_spin_ms >= ROS_PUBLISH_PERIOD_MS) 
	{
		for (int i=0; i<8; i++)
		{
			feedback_msg.position[i] = _jstate_pos[i];
			feedback_msg.velocity[i] = _jstate_vel[i];
			feedback_msg.effort[i]   = _jstate_eff[i];
			// feedback_msg.effort[i]   = floorf(map(_jstate_eff[i], -100.0, 100.0, -255, 255));
		}
		feedback_msg.header.stamp = nh.now();
		state_publisher.publish( &feedback_msg );
		nh.spinOnce();
		since_last_spin_ms = since_last_spin_ms - ROS_PUBLISH_PERIOD_MS;
	}
}
