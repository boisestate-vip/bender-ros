#include <bender_hardware/bender_hardware.h>

BenderHardware::BenderHardware(ros::NodeHandle& nh) : nh_(nh)
{
    init();   
}


BenderHardware::~BenderHardware()
{
}


void BenderHardware::init()
{
    nh_.getParam("bender/hardware_interface/joints", joint_names_);
    if (joint_names_.size() == 0)
    {
        ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
    }
    num_joints_ = joint_names_.size();

    // Resize vectors
    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);
	joint_effort_command_.resize(num_joints_);

    // Initialize controller
    for (int i = 0; i < num_joints_; ++i)
    {
        // Create joint state interface
        JointStateHandle jointStateHandle(&joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        JointLimits limits;
        SoftJointLimits softLimits;
        if (getJointLimits(joint.name, nh_, limits) == false) {
            ROS_ERROR_STREAM("Cannot set joint limits for " << joint.name);
        } else {
            PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
            positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
        }
        position_joint_interface_.registerHandle(jointPositionHandle);

        // Create velocity joint interface
        JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        effort_joint_interface_.registerHandle(jointVelocityHandle);

        // Create effort joint interface
        JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
        effort_joint_interface_.registerHandle(jointEffortHandle);

    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&positionJointSoftLimitsInterface);
}