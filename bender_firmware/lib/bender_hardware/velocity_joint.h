#ifndef BENDER_FIRMWARE_VELOCITY_JOINT_H
#define BENDER_FIRMWARE_VELOCITY_JOINT_H

#include "Arduino.h"

#include <ros/ros.h>
#include <urdf/model.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

class VelocityJoint
{
    public:
        VelocityJoint(int vrPin, int zfPin, int interrputPin);
        void setState(double state);
        void getState(double &state);
        void setTarget(double target);
        void getTarget(double &target);
        void actuate();
        
    private:
        int vr_speed_pin_;
        int zf_dir_pin_;
        int tach_pin_;      //See https://github.com/PaulStoffregen/FreqMeasureMulti for measuring frequency
        double upper_limit_;
        double lower_limit_;
        double target_;
        double state_;
        double state_last_;
        double effort_;
}; // class VelocityJoint