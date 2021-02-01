/**
 *
 *  \file
 *  \brief      This class is based on jackal_base package for Clearpath's Jackal hardware
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author     Wankun Sirichotiyakul <wankunsirichotiyakul@u.boisestate.edu>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */


#include <boost/assign.hpp>
#include "bender_base/bender_hardware.h"


namespace bender_base
{

BenderHardware::BenderHardware()
{
    ros::V_string joint_names = boost::assign::list_of
        ("wheel_lf_joint")("wheel_rf_joint")("wheel_lh_joint")("wheel_rh_joint")
        ("leg_lf_joint")("leg_rf_joint")("leg_lh_joint")("leg_rh_joint");

    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
            &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);
        if (i >= 4) {
            hardware_interface::JointHandle joint_handle(
                joint_state_handle, &joints_[i].command);
            position_joint_interface_.registerHandle(joint_handle);
        } else {
            hardware_interface::JointHandle joint_handle(
                joint_state_handle, &joints_[i].command);
            velocity_joint_interface_.registerHandle(joint_handle);
        }
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);

    feedback_sub_ = nh_.subscribe("feedback", 1, &BenderHardware::feedbackCallback, this);
    cmd_drive_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("cmd_drive", 1);
	cmd_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	cmd_msg_.layout.dim[0].label = "joint_targets";
	cmd_msg_.layout.dim[0].size = 8;
    cmd_msg_.layout.dim[0].stride = 1;
}


BenderHardware::~BenderHardware()
{
}


void BenderHardware::copyJointsFromHardware()
{
    boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
    if (feedback_msg_ && feedback_msg_lock)
    {
		for (int i = 0; i < 8; i++)
		{
			joints_[i].position = feedback_msg_->position[i];
			joints_[i].velocity = feedback_msg_->velocity[i];
			joints_[i].effort   = feedback_msg_->effort[i];  
		}
	}
}


void BenderHardware::publishDriveToMCU()
{
	cmd_msg_.data.clear();
	for (int i = 0; i < 8; i++)
	{
		cmd_msg_.data.push_back( (float) joints_[i].command );
	}
	cmd_drive_pub_.publish(cmd_msg_);
}


void BenderHardware::feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	// Update the feedback message pointer to point to the current message. Block
	// until the control thread is not using the lock.
	boost::mutex::scoped_lock lock(feedback_msg_mutex_);
	feedback_msg_ = msg;
}

}  // namespace bender_base
