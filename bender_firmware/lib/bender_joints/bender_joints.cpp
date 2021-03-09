#include "bender_joints.h"


/*********************************************************************
 * GenericJoint implementations
*********************************************************************/
GenericJoint::GenericJoint(float p, float i, float d, 
                           float lowerLimit, float upperLimit, float effortLimit) : 
    pid_controller_(p,i,d,-50.0f,50.0f),
    enabled_(false),
    upper_limit_(upperLimit),
    lower_limit_(lowerLimit),
    effort_limit_(effortLimit),
    target_(0.0),
    state_(0.0),
    state_last_(0.0),
    effort_(0.0),
    error_(0.0)
{
}

void GenericJoint::update(unsigned long dt_ms)
{
    error_ = clamp(target_, lower_limit_, upper_limit_) - state_;    
    if (dt_ms <= 100)
    {
        effort_ = clamp(pid_controller_.computeCommand(error_, dt_ms),
                        -effort_limit_, effort_limit_);
    }
}


/*********************************************************************
 * PositionJoint implementations
*********************************************************************/
PositionJoint::PositionJoint(uint8_t encAPin, uint8_t encBPin, uint8_t pwmPin, uint8_t dirPin,
                             unsigned int encoderPPR, float p, float i, float d) :
    GenericJoint(p, i, d, -M_PI_2, M_PI_2),
    enc_a_pin_(encAPin),
    enc_b_pin_(encBPin),
    pwm_pin_(pwmPin),
    dir_pin_(dirPin),
    pulse_per_rev_(static_cast<float>(encoderPPR)),
    encoder_(encAPin,encBPin)
{ 
    pinMode(pwm_pin_, OUTPUT);
    pinMode(dir_pin_, OUTPUT);
}

void PositionJoint::update(unsigned long dt_ms)
{
    setState(encoder_.read() / pulse_per_rev_ * 2 * M_PI);
    // GenericJoint::update(dt_ms);
    angles::shortest_angular_distance_with_limits(state_,
                                                  target_,
                                                  lower_limit_,
                                                  upper_limit_,
                                                  error_);
    
    if (dt_ms <= 100)
    {
        effort_ = clamp(pid_controller_.computeCommand(error_, dt_ms),
                        -effort_limit_, effort_limit_);
    }
}

void PositionJoint::getState(float &state)
{
    setState(encoder_.read() / pulse_per_rev_ * 2 * M_PI);
    state = state_;
}

void PositionJoint::actuate()
{
    if (enabled_) 
    {
        if (effort_ > 0) 
        { 
            digitalWriteFast(dir_pin_, HIGH); 
            analogWrite(pwm_pin_, floorf(map(effort_,0.0f,100.0f,0,255)));
        }
        else
        {
            digitalWriteFast(dir_pin_, LOW);
            analogWrite(pwm_pin_, floorf(map(-effort_,0.0f,100.0f,0,255)));
        }
    }
}

void PositionJoint::stop()
{
    effort_ = 0.0;
    analogWrite(pwm_pin_, 0);
}


/*********************************************************************
 * VelocityJoint implementations
*********************************************************************/
VelocityJoint::VelocityJoint(uint8_t vrPin, uint8_t zfPin, uint8_t tachPin, uint8_t powerPin, 
                             unsigned int tachPPR, float p, float i, float d, int rpmLimit) :
    GenericJoint(p, i, d, -rpmLimit*RPM_TO_RAD_S, rpmLimit*RPM_TO_RAD_S),
    vr_speed_pin_(vrPin),
    zf_dir_pin_(zfPin),
    tach_pin_(tachPin),
    power_pin_(powerPin),
    pulse_per_rev_(static_cast<float>(tachPPR))
{
    pinMode(vr_speed_pin_, OUTPUT);
    pinMode(zf_dir_pin_, OUTPUT);
    pinMode(power_pin_, OUTPUT);
    digitalWriteFast(power_pin_, LOW);
    since_last_interrupt_ = 0;
    since_last_sign_change_ = 0;
}

void VelocityJoint::update(unsigned long dt_ms)
{
    pulsesToRPM();
    setState(direction_*rpm_*RPM_TO_RAD_S);  // convert to rad/s
    GenericJoint::update(dt_ms);
}

void VelocityJoint::getState(float &state)
{
    pulsesToRPM();
    setState(direction_*rpm_*RPM_TO_RAD_S);
    state = state_;
}

void VelocityJoint::setTarget(float target)
{
    direction_last_ = direction_;
    if (target > 0) {direction_ =  1;}
    else if (target < 0) {direction_ = -1;}
    target_ = target;
}

void VelocityJoint::controllerPowerCycle_()
{
    digitalWriteFast(power_pin_, LOW);
    delayMicroseconds(100);
    digitalWriteFast(power_pin_, HIGH);
}

void VelocityJoint::actuate()
{
    if (enabled_)
    {
        if (direction_last_*direction_ < 0)
        {
            controllerPowerCycle_();
        }
        if (target_ > 0) 
        { 
            // digitalWriteFast(power_pin_, HIGH);
            digitalWriteFast(zf_dir_pin_, HIGH); 
            analogWrite(vr_speed_pin_, floorf(map(abs(effort_),0.0f,100.0f,0,255)));
        }
        else if (target_ < 0) 
        { 
            // digitalWriteFast(power_pin_, HIGH);
            digitalWriteFast(zf_dir_pin_, LOW);  
            analogWrite(vr_speed_pin_, floorf(map(abs(effort_),0.0f,100.0f,0,255)));
        }
        else
        {
            // digitalWriteFast(power_pin_, LOW);
            analogWrite(vr_speed_pin_, LOW);
        }
        // if (direction_last_*direction_ < 0)
        // {
            // controllerPowerCycle_();
        // }
        // if (effort_ > 0) 
        // { 
        //     // digitalWriteFast(power_pin_, HIGH);
        //     digitalWriteFast(zf_dir_pin_, HIGH); 
        //     analogWrite(vr_speed_pin_, floorf(map(effort_,0.0f,100.0f,0,255)));
        // }
        // else if (effort_ < 0) 
        // { 
        //     // digitalWriteFast(power_pin_, HIGH);
        //     digitalWriteFast(zf_dir_pin_, LOW);  
        //     analogWrite(vr_speed_pin_, floorf(map(-effort_,0.0f,100.0f,0,255)));
        // }
        // else
        // {
        //     // digitalWriteFast(power_pin_, LOW);
        //     analogWrite(vr_speed_pin_, LOW);
        // }
    }
}

void VelocityJoint::stop()
{
    digitalWriteFast(power_pin_, LOW);
    effort_ = 0.0;
    analogWrite(vr_speed_pin_, 0);
}

uint8_t VelocityJoint::getInterruptPin()
{
    return tach_pin_;
}

void VelocityJoint::interruptHandle()
{
    pulses_ += 1;
    interval_ += since_last_interrupt_;
    since_last_interrupt_ = 0;
}

void VelocityJoint::pulsesToRPM()
{
    noInterrupts();
    
    /* 
     * If more than 250 ms has elapsed since last pulse was registered, 
     * wheel isn't moving (< 5.33 RPM).
     */
    if (since_last_interrupt_ >= 250000)  
    {
        pulses_   = 0;
        interval_ = 0;
        rpm_ = 0.0;
    }
    /*
     * Do RPM calculation if we have collected data from 3 pulses or more
     */
    else if (pulses_ > 3)
    {
        rpm_ = (float) pulses_ / pulse_per_rev_ / ((float) interval_ / 1000000.0f) * 60.0f;
        pulses_ = 0;
        interval_ = 0;
    }

    interrupts();
}