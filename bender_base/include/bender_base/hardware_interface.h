#ifndef BENDER_BASE_HARDWARE_INTERFACE_H
#define BENDER_BASE_HARDWARE_INTERFACE_H

#include <boost/thread.hpp>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <socketcan_interface/socketcan.h>

#include <odrive_can_ros/can_simple.hpp>

#include <ros/ros.h>
#include <urdf/model.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>


namespace bender_base
{

using namespace odrive_can_ros;
using namespace std::literals::chrono_literals;

class BenderHardware : public hardware_interface::RobotHW 
{
    public:
        BenderHardware();
        ~BenderHardware();
        void read();
        void write();

    protected:
        void feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg);

        ros::NodeHandle nh_;
        ros::Subscriber feedback_sub_;
        ros::Publisher cmd_drive_pub_;

        // Interfaces
        hardware_interface::JointStateInterface      joint_state_interface_;
        hardware_interface::PositionJointInterface   position_joint_interface_;
        hardware_interface::VelocityJointInterface   velocity_joint_interface_;

        // These are mutated on the controls thread only.
        struct Joint
        {
            double position;
            double velocity;
            double effort;
            double command;

            Joint() : position(0), velocity(0), effort(0), command(0)
            {
            }
        }
        joints_[8];

        // This pointer is set from the ROS thread.
        std_msgs::Float32MultiArray cmd_msg_;
        sensor_msgs::JointState::ConstPtr feedback_msg_;
        boost::mutex feedback_msg_mutex_;

        // CAN related
        CANSimple canbus_;
        std::vector<std::string> can_node_names;


}; // class

}  // namespace bender_base

#endif // BENDER_BASE_HARDWARE_INTERFACE_H