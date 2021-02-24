#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include "pid.h"
#include "bender_joints.h"

#define PID_UPDATE_PERIOD_MS 1
#define ROS_PUBLISH_PERIOD_MS 5
#define CMD_RECEIVE_TIMEOUT_MS 100
#define PLANETARY_PPR 6672
#define HUB_PPR 45
#define MAX_THROTTLE_PERCENT 30


// Robot's joints
PositionJoint pos_joints[4] = {
	PositionJoint(2, 3, 35, 16, PLANETARY_PPR, 20.0, 0.0, 5.0), // leg_lf_joint
	PositionJoint(8, 9, 38, 19, PLANETARY_PPR, 20.0, 0.0, 5.0), // leg_rf_joint
	PositionJoint(4, 5, 36, 17, PLANETARY_PPR, 20.0, 0.0, 5.0), // leg_lh_joint
	PositionJoint(6, 7, 37, 18, PLANETARY_PPR, 20.0, 0.0, 5.0)  // leg_rh_joint
};
VelocityJoint vel_joints[4] = {
	VelocityJoint(10, 11, 12, 20, HUB_PPR, 5.0, 0.0, 0.0), // wheel_lf_joint
	VelocityJoint(30, 26, 34, 23, HUB_PPR, 5.0, 0.0, 0.0), // wheel_rf_joint
	VelocityJoint(14, 13, 15, 21, HUB_PPR, 5.0, 0.0, 0.0), // wheel_lh_joint
	VelocityJoint(29, 28, 27, 22, HUB_PPR, 5.0, 0.0, 0.0)  // wheel_rh_joint
};
/*
 * The following is an unfortunate consequence of Arduino's
 * attachInterrupt function not supporting any way to pass a 
 * pointer or other context to the attached function.
 */
void velJoint0ISR(void) { vel_joints[0].interruptHandle(); }
void velJoint1ISR(void) { vel_joints[1].interruptHandle(); }
void velJoint2ISR(void) { vel_joints[2].interruptHandle(); }
void velJoint3ISR(void) { vel_joints[3].interruptHandle(); }


// Timers
elapsedMillis since_last_receipt_ms;
elapsedMillis since_last_update_ms;
elapsedMillis since_last_spin_ms;


// ROS Nodehandle
ros::NodeHandle nh;


// ROS JointState publisher
char *_jstate_name[] = {
	"wheel_lf_joint", "wheel_rf_joint", "wheel_lh_joint", "wheel_rh_joint",
	"leg_lf_joint", "leg_rf_joint", "leg_lh_joint", "leg_rh_joint"
};
float _jstate_pos[8] = {0,0,0,0,0,0,0,0};
float _jstate_vel[8] = {0,0,0,0,0,0,0,0};
float _jstate_eff[8] = {0,0,0,0,0,0,0,0};
sensor_msgs::JointState feedback_msg;
ros::Publisher state_publisher("feedback", &feedback_msg);


// ROS cmd_drive subscriber
void updateCmd(const std_msgs::Float32MultiArray &cmd_msg)
{
	since_last_receipt_ms = 0;
	if (cmd_msg.layout.dim[0].size == 8)
	{
		for (unsigned int i=0; i<8; i++)
		{
			float target = cmd_msg.data[i];
			if (i < 4) {
				vel_joints[i].setTarget(target);
			} else if (4 <= i && i < 8) {
				pos_joints[i-4].setTarget(target);
			}
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

	attachInterrupt(vel_joints[0].getInterruptPin(), velJoint0ISR, RISING);
	attachInterrupt(vel_joints[1].getInterruptPin(), velJoint1ISR, RISING);
	attachInterrupt(vel_joints[2].getInterruptPin(), velJoint2ISR, RISING);
	attachInterrupt(vel_joints[3].getInterruptPin(), velJoint3ISR, RISING);

	/** Enable the motors **/
	for (int i=0; i<4; i++)
	{
		// vel_joints[i].enable();
		// pos_joints[i].enable();
		vel_joints[i].setEffortLimit(MAX_THROTTLE_PERCENT);
		pos_joints[i].setEffortLimit(MAX_THROTTLE_PERCENT);
	}
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
			vel_joints[i].actuate();
			pos_joints[i].update(since_last_update_ms);
			pos_joints[i].actuate();
		}
		since_last_update_ms = 0;
	}

	/** Publish via rosserial at a specified rate **/
	if (since_last_spin_ms >= ROS_PUBLISH_PERIOD_MS) 
	{
		for (int i=0; i<8; i++)
		{
			if (i < 4) {
				vel_joints[i].getState(_jstate_vel[i]);
				vel_joints[i].getEffort(_jstate_eff[i]);
			} else if (4 <= i && i < 8) {
				pos_joints[i-4].getState(_jstate_pos[i]);
				pos_joints[i-4].getEffort(_jstate_eff[i]);
			}
			feedback_msg.position[i] = _jstate_pos[i];
			feedback_msg.velocity[i] = _jstate_vel[i];
			feedback_msg.effort[i]   = _jstate_eff[i];
		}
		feedback_msg.header.stamp = nh.now();
		state_publisher.publish( &feedback_msg );
		nh.spinOnce();
		if (since_last_spin_ms >= 3*ROS_PUBLISH_PERIOD_MS)
		{
			since_last_spin_ms = 0;
		}
		else 
		{
			since_last_spin_ms = since_last_spin_ms - ROS_PUBLISH_PERIOD_MS;
		}
	}
}
