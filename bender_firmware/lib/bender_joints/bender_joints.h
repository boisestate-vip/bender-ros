#ifndef BENDER_FIRMWARE_BENDER_JOINTS_H
#define BENDER_FIRMWARE_BENDER_JOINTS_H

#define ANALOGWRITE_MAX 1023.0f
#define ANALOGWRITE_MIN 25.0f

#define WHEEL_POWER_PIN_OFF 1
#define WHEEL_DIR_PIN_FORWARD 1

#include <Arduino.h>
#include <Encoder.h>

#include "angles.h"
#include "pid.h"
#include "bender_utils.h"

class GenericJoint
{
    public:
        GenericJoint(float p, float i, float d, 
                     float lowerLimit, float upperLimit, 
                     float effortLower=-100.0, float effortUpper=100.0);
        void setState(float state)    { state_last_ = state_; state_ = state; }
        void getState(float &state)   { state = state_; }
        void setTarget(float target)  { target_ = target; }
        void getTarget(float &target) { target = target_; }
        void getEffort(float &effort) { effort = effort_; }
        void setEffortUpperLimit(float val) { effort_upper_ = val; }
        void setEffortLowerLimit(float val) { effort_lower_ = val; }
        void getError(float &error)   { error = error_; }
        void setGains(float p, float i, float d)    { pid_controller_.setGains(p,i,d); }
        void getGains(float &p, float &i, float &d) { pid_controller_.getGains(p,i,d); }
        void enable()  { enabled_ = true; }
        void disable() { enabled_ = false; }
        bool isEnabled() { return enabled_; }
        void update(unsigned long dt_ms);
        virtual void actuate() { };
        virtual void stop() { };

    protected:
        Pid pid_controller_;
        bool enabled_;
        const float upper_limit_;
        const float lower_limit_;
        float effort_upper_;
        float effort_lower_;
        float target_;
        float state_;
        float state_last_;
        float effort_;
        float effort_last_;
        float error_;
}; // class GenericJoint


class PositionJoint : public GenericJoint
{
    public:
        PositionJoint(uint8_t encAPin, uint8_t encBPin, uint8_t pwmPin, uint8_t dirPin, 
                      unsigned int encoderPPR, float p=0.0, float i=0.0, float d=0.0);
        void update(unsigned long dt_ms);
        void getState(float &state);
        void actuate() override;
        void stop() override;

    private:
        uint8_t enc_a_pin_;
        uint8_t enc_b_pin_;
        uint8_t pwm_pin_;
        uint8_t dir_pin_;
        const float pulse_per_rev_;
        Encoder encoder_;
}; // class PositionJoint


class VelocityJoint : public GenericJoint
{
    public:
        VelocityJoint(uint8_t vrPin, uint8_t zfPin, uint8_t tachPin, uint8_t powerPin, 
                      unsigned int tachPPR, float p=0.0, float i=0.0, float d=0.0, int rpmLimit=100);
        
        void update(unsigned long dt_ms);
        void getState(float &state);
        void getEffort(float &effort);
        void setTarget(float target);
        void enable();
        void actuate() override;
        void stop() override;
        uint8_t getInterruptPin();
        void interruptHandle();
        void pulsesToRPM();
    
    private:
        void controllerPowerCycle_();
        uint8_t vr_speed_pin_;
        uint8_t zf_dir_pin_;
        uint8_t tach_pin_;
        uint8_t power_pin_;      
        // Some notes about calling attachInterrupt() with member functions
        // https://atadiat.com/en/e-arduino-trick-share-interrupt-service-routines-between-libraries-application/
        // https://www.onetransistor.eu/2019/05/arduino-class-interrupts-and-callbacks.html
        // https://forum.arduino.cc/index.php?topic=365383.0
        const float pulse_per_rev_;
        volatile unsigned long int pulses_ = 0;
        unsigned long int interval_ = 0;
        int8_t direction_ = 0;
        int8_t direction_last_ = 0;
        float rpm_  = 0.0f;
        elapsedMicros since_last_interrupt_;
        elapsedMillis since_last_sign_change_;
}; // class VelocityJoint

#endif // BENDER_FIRMWARE_BENDER_JOINTS_H