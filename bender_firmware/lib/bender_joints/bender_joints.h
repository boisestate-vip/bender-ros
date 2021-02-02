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
                     float lowerLimit, float upperLimit, float effortLimit=100.0);
        void setState(float state)    { state_ = state; }
        void getState(float &state)   { state = state_; }
        void setTarget(float target)  { target_ = target; }
        void getTarget(float &target) { target = target_; }
        void getEffort(float &effort) { effort = effort_; }
        void getError(float &error)   { error = error_; }
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
        float error_;
}; // class GenericJoint


class PositionJoint : public GenericJoint
{
    public:
        PositionJoint(int encAPin, int encBPin, int pwmPin, int dirPin, 
                      float p=0.0, float i=0.0, float d=0.0);
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
        void actuate();
        void stop();
    
    private:
        int vr_speed_pin_;
        int zf_dir_pin_;
        int tach_pin_;      // Some notes about calling attachInterrupt() with member functions
                            // https://atadiat.com/en/e-arduino-trick-share-interrupt-service-routines-between-libraries-application/
                            // https://www.onetransistor.eu/2019/05/arduino-class-interrupts-and-callbacks.html
                            // https://forum.arduino.cc/index.php?topic=365383.0
}; // class VelocityJoint

#endif // BENDER_FIRMWARE_BENDER_JOINTS_H