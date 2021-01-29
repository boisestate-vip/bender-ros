#ifndef BENDER_FIRMWARE_POSITION_JOINT_H
#define BENDER_FIRMWARE_POSITION_JOINT_H

#include "Arduino.h"


class PositionJoint
{
    public:
        PositionJoint(int encAPin, int encBPin, int pwmPin, int dirPin);
        void setState(double state);
        void getState(double &state);
        void setTarget(double target);
        void getTarget(double &target);
        void actuate();

    private:
        int enc_a_pin_;
        int enc_b_pin_;
        int pwm_pin_;
        int dir_pin_;
        double upper_limit_;
        double lower_limit_;
        double target_;
        double state_;
        double state_last_;
        double effort_;
}; // class PositionJoint