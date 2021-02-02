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
        PositionJoint(uint8_t encAPin, uint8_t encBPin, uint8_t pwmPin, uint8_t dirPin, 
                      float p=0.0, float i=0.0, float d=0.0);
        void update(unsigned long dt_ms);
        void actuate();
        void stop();

    private:
        uint8_t enc_a_pin_;
        uint8_t enc_b_pin_;
        uint8_t pwm_pin_;
        uint8_t dir_pin_;
        Encoder encoder_;
}; // class PositionJoint


class VelocityJoint : public GenericJoint
{
    public:
        VelocityJoint(uint8_t vrPin, uint8_t zfPin, uint8_t interrputPin, 
                      float p=0.0, float i=0.0, float d=0.0, float velLimit=5.0);
        
        void update(unsigned long dt_ms);
        void actuate();
        void stop();
        uint8_t getInterruptPin();
        void interruptHandle();
        float pulsesToRPM();
    
    private:
        uint8_t vr_speed_pin_;
        uint8_t zf_dir_pin_;
        uint8_t tach_pin_;      
        // Some notes about calling attachInterrupt() with member functions
        // https://atadiat.com/en/e-arduino-trick-share-interrupt-service-routines-between-libraries-application/
        // https://www.onetransistor.eu/2019/05/arduino-class-interrupts-and-callbacks.html
        // https://forum.arduino.cc/index.php?topic=365383.0
        int pulses_ = 0;
        float rpm_  = 0.0f;
        unsigned long interval_ = 0;
        elapsedMicros since_last_interrupt_;
}; // class VelocityJoint

#endif // BENDER_FIRMWARE_BENDER_JOINTS_H