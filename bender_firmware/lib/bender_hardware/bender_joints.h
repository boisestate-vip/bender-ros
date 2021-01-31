#ifndef BENDER_FIRMWARE_BENDER_JOINTS_H
#define BENDER_FIRMWARE_BENDER_JOINTS_H

#include <Arduino.h>
#include <Encoder.h>

#include "angles.h"
#include "pid.h"

class GenericJoint
{
    public:
        GenericJoint(float p, float i, float d, 
                     float lowerLimit, float upperLimit, float effortLimit=1.0);
        void setState(float state)    { state_ = state;   }
        void getState(float &state)   { state = state_;   }
        void setTarget(float target)  { target_ = target; }
        void getTarget(float &target) { target = target_; }
        void setGains(float p, float i, float d)    { pid_controller_.setGains(p,i,d); }
        void getGains(float &p, float &i, float &d) { pid_controller_.getGains(p,i,d); }
        void enable()  { enabled_ = true; }
        void disable() { enabled_ = false; }
        void update(unsigned long dt_ms);
        virtual void actuate() { };
        virtual void stop() { };

    protected:
        Pid pid_controller_;
        bool enabled_;
        float upper_limit_;
        float lower_limit_;
        float target_;
        float state_;
        float effort_;
        float effort_limit_;
}; // class GenericJoint


class PositionJoint : public GenericJoint
{
    public:
        PositionJoint(int encAPin, int encBPin, int pwmPin, int dirPin, 
                      float p=0.0, float i=0.0, float d=0.0);
        // ~PositionJoint();
        void update(unsigned long dt_ms);
        void actuate();
        void stop();

    private:
        int enc_a_pin_;
        int enc_b_pin_;
        int pwm_pin_;
        int dir_pin_;
        Encoder encoder_;
}; // class PositionJoint


class VelocityJoint : public GenericJoint
{
    public:
        VelocityJoint(int vrPin, int zfPin, int interrputPin, 
                      float p=0.0, float i=0.0, float d=0.0, float velLimit=5.0);
        // ~VelocityJoint();
        void actuate();
        void stop();
        
    private:
        int vr_speed_pin_;
        int zf_dir_pin_;
        int tach_pin_;      // See https://github.com/PaulStoffregen/FreqMeasureMulti for measuring frequency
}; // class VelocityJoint

#endif // BENDER_FIRMWARE_BENDER_JOINTS_H