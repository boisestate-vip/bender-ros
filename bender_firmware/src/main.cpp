#define PID_UPDATE_PERIOD_US 200
#define ROS_PUBLISH_PERIOD_MS 10
#define CMD_RECEIVE_TIMEOUT_MS 200
#define PLANETARY_PPR 6672
#define HUB_PPR 45
#define MAX_LEG_THROTTLE_PERCENT 40
#define MAX_WHEEL_THROTTLE_PERCENT 100

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include "pid.h"
#include "bender_joints.h"

// Robot's joints
PositionJoint pos_joints[4] = {
	PositionJoint(2, 3, 35, 32, PLANETARY_PPR, 25.0, 0.1, 10.0), // leg_lf_joint
	PositionJoint(8, 9, 38, 19, PLANETARY_PPR, 25.0, 0.1, 10.0), // leg_rf_joint
	PositionJoint(4, 5, 36, 17, PLANETARY_PPR, 25.0, 0.1, 10.0), // leg_lh_joint
	PositionJoint(6, 7, 37, 18, PLANETARY_PPR, 25.0, 0.1, 10.0)  // leg_rh_joint
};

// Timers
elapsedMillis since_last_receipt_ms;
elapsedMicros since_last_update_us;
elapsedMillis since_last_spin_ms;


// ROS Nodehandle
ros::NodeHandle nh;


// ROS JointState publisher
char *_jstate_name[] = {
	"leg_lf_joint", "leg_rf_joint", "leg_lh_joint", "leg_rh_joint"
};
float _jstate_pos[4] = {0,0,0,0};
float _jstate_vel[4] = {0,0,0,0};
float _jstate_eff[4] = {0,0,0,0};
sensor_msgs::JointState feedback_msg;
ros::Publisher state_publisher("/bender_teensy_serial/feedback", &feedback_msg);


// ROS cmd_drive subscriber
void updateCmd(const std_msgs::Float32MultiArray &cmd_msg)
{
	since_last_receipt_ms = 0;
	if (cmd_msg.layout.dim[0].size == 4)
	{
		for (unsigned int i=0; i<4; i++)
		{
			float target = cmd_msg.data[i];
			pos_joints[i].setTarget(target);
		}
	}
}
ros::Subscriber<std_msgs::Float32MultiArray> cmd_subscriber("/bender_teensy_serial/cmd_drive", updateCmd);


void setup()
{
	nh.initNode();
	feedback_msg.name            = _jstate_name;
	feedback_msg.position        = _jstate_pos;
	feedback_msg.velocity        = _jstate_vel;
	feedback_msg.effort          = _jstate_eff;
	feedback_msg.name_length     = 4;
	feedback_msg.position_length = 4;
	feedback_msg.velocity_length = 4;
	feedback_msg.effort_length   = 4;
	nh.advertise(state_publisher);
	nh.subscribe(cmd_subscriber);

#ifdef USE_PWM_10BIT
	analogWriteFrequency(35, 18000.0f);
	analogWriteFrequency(36, 18000.0f);
	analogWriteFrequency(37, 18000.0f);
	analogWriteFrequency(38, 18000.0f);
	analogWriteResolution(10);
#endif

	since_last_receipt_ms = 0;
	since_last_update_us = 0;
	since_last_spin_ms = 0;

	/** Enable the motors **/
	for (int i=0; i<4; i++)
	{
		pos_joints[i].enable();
		pos_joints[i].setEffortUpperLimit(MAX_LEG_THROTTLE_PERCENT);
		pos_joints[i].setEffortLowerLimit(-MAX_LEG_THROTTLE_PERCENT);
	}
}


void loop()
{
	/** If it has been too long since last command was received, stop the motors **/
	if (since_last_receipt_ms >= CMD_RECEIVE_TIMEOUT_MS)
	{
		for (int i=0; i<4; i++)
		{
			pos_joints[i].stop();
		}
	}	
	/** Update the motor commands at a specifeid rate **/
	else if (since_last_update_us >= PID_UPDATE_PERIOD_US) 
	{
		for (int i=0; i<4; i++)
		{
			pos_joints[i].update(since_last_update_us);
			pos_joints[i].actuate();
		}
		since_last_update_us = 0;
	}

	/** Publish via rosserial at a specified rate **/
	if (since_last_spin_ms >= ROS_PUBLISH_PERIOD_MS) 
	{
		for (int i=0; i<4; i++)
		{
			pos_joints[i].getState(_jstate_pos[i]);
			pos_joints[i].getEffort(_jstate_eff[i]);
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
