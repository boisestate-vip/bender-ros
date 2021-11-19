#include <boost/assign.hpp>
#include "bender_base/hardware_interface.h"


namespace bender_base
{

BenderHardware::BenderHardware()
{
    ros::V_string joint_names = boost::assign::list_of
        ("wheel_rf_joint")("wheel_rh_joint")("wheel_lf_joint")("wheel_lh_joint")
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

    feedback_sub_ = nh_.subscribe("/bender_teensy_serial/feedback", 1, &BenderHardware::feedbackCallback, this);
    cmd_drive_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/bender_teensy_serial/cmd_drive", 1);
	cmd_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	cmd_msg_.layout.dim[0].label = "joint_targets";
	cmd_msg_.layout.dim[0].size = 4;
    cmd_msg_.layout.dim[0].stride = 1;

    // CAN setup
    std::string can_device = nh_.param<std::string>("can_device", "can0");
    can_node_names.push_back("wheel_rf_joint");
    can_node_names.push_back("wheel_rh_joint");
    can_node_names.push_back("wheel_lf_joint");
    can_node_names.push_back("wheel_lh_joint");
    const unsigned short id0(nh_.param("wheel_rf_joint_node_id", 0));
    const unsigned short id1(nh_.param("wheel_rh_joint_node_id", 1));
    const unsigned short id2(nh_.param("wheel_lf_joint_node_id", 2));
    const unsigned short id3(nh_.param("wheel_lh_joint_node_id", 3));
    if ( !( canbus_.add_axis(id0, "wheel_rf_joint") &&
            canbus_.add_axis(id1, "wheel_rh_joint") &&
            canbus_.add_axis(id2, "wheel_lf_joint") &&
            canbus_.add_axis(id3, "wheel_lh_joint") ) ) {
        ROS_FATAL("Failed to create one or more axis. Aborting.\n");
    }
    can::ThreadedSocketCANInterfaceSharedPtr driver = 
        std::make_shared<can::ThreadedSocketCANInterface>();
    if (!driver->init(can_device, 0, can::NoSettings::create()))
    {
        ROS_FATAL("Failed to initialize can_device at %s\n", can_device.c_str());
    }
    can::StateListenerConstSharedPtr state_listener = driver->createStateListener(
        [&driver](const can::State& s) {
            std::string err;
            driver->translateError(s.internal_error, err);
            fprintf(stderr, "CAN Device error: %s, asio: %s.\n", 
                err.c_str(), s.error_code.message().c_str());
        }
    );
    // Pass the SocketCAN handle to master
    canbus_.init(driver);
    std::this_thread::sleep_for(500ms);
    for (auto& name : can_node_names)
    {
        canbus_.clear_errors(canbus_.axis(name));
        canbus_.set_axis_requested_state(canbus_.axis(name), AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    }
}


BenderHardware::~BenderHardware()
{
    for (auto& name : can_node_names)
    {
        canbus_.set_input_vel(canbus_.axis(name), 0.0f);
        canbus_.set_axis_requested_state(canbus_.axis(name), AxisState::AXIS_STATE_IDLE);
    }
}


void BenderHardware::read()
{
    // CAN Bus
    for (auto& name : can_node_names)
    {
        const int node_id = canbus_.axis(name).node_id;
        joints_[node_id].position = canbus_.axis(name).pos_enc_estimate * 2.0 * M_PI;
        joints_[node_id].velocity = canbus_.axis(name).vel_enc_estimate * 2.0 * M_PI / 60.0;
        joints_[node_id].effort = joints_[node_id].command;
    }

    // Serial
    boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
    if (feedback_msg_ && feedback_msg_lock)
    {
		for (int i = 4; i < 8; i++)
		{
			joints_[i].position = feedback_msg_->position[i-4];
			joints_[i].velocity = feedback_msg_->velocity[i-4];
			joints_[i].effort   = feedback_msg_->effort[i-4];  
		}
	}
    
}


void BenderHardware::write()
{
	// CAN Bus
     for (auto& name : can_node_names)
    {
        const int node_id = canbus_.axis(name).node_id;
        const float cmd = joints_[node_id].command * 60.0 / 2.0 / M_PI;
        canbus_.set_input_vel(canbus_.axis(name), joints_[node_id].command);
    }
    
    cmd_msg_.data.clear();
	for (int i = 4; i < 8; i++)
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
