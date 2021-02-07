#include "bender_joints.h"


float clamp(const float val, const float min_val, const float max_val)
{
    return min(max(val, min_val), max_val);
}

/*********************************************************************
 * GenericJoint implementations
*********************************************************************/
GenericJoint::GenericJoint(float p, float i, float d, 
                           float lowerLimit, float upperLimit, float effortLimit) : 
    pid_controller_(p,i,d),
    enabled_(false),
    upper_limit_(upperLimit),
    lower_limit_(lowerLimit),
    target_(0.0),
    state_(0.0),
    effort_(0.0),
    effort_limit_(effortLimit),
    error_(0.0)
{
}

void GenericJoint::update(unsigned long dt_ms)
{
    if      (target_ <= lower_limit_)  { error_ = lower_limit_ - state_; } 
    else if (target_ >= upper_limit_)  { error_ = upper_limit_ - state_; } 
    else                               { error_ = target_ - state_; }
    
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
                             float p, float i, float d) :
    GenericJoint(p, i, d, -M_PI_2, M_PI_2),
    enc_a_pin_(encAPin),
    enc_b_pin_(encBPin),
    pwm_pin_(pwmPin),
    dir_pin_(dirPin),
    encoder_(encAPin,encBPin) 
{ 
    pinMode(pwm_pin_, OUTPUT);
    pinMode(dir_pin_, OUTPUT);
}

void PositionJoint::update(unsigned long dt_ms)
{
    setState(encoder_.read());
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
    state_ = encoder_.read();
    state = state_;
}

void PositionJoint::actuate()
{
    if (enabled_) 
    {
        if (effort_ > 0) 
        { 
            digitalWrite(dir_pin_, HIGH); 
            analogWrite(pwm_pin_, floorf(map(effort_,0.0f,100.0f,0,255)));
        }
        else
        {
            digitalWrite(dir_pin_, LOW);
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
VelocityJoint::VelocityJoint(uint8_t vrPin, uint8_t zfPin, uint8_t tachPin, 
                             float p, float i, float d, int rpmLimit) :
    GenericJoint(p, i, d, -rpmLimit*RPM_TO_RAD_S, rpmLimit*RPM_TO_RAD_S),
    vr_speed_pin_(vrPin),
    zf_dir_pin_(zfPin),
    tach_pin_(tachPin)
{
    pinMode(vr_speed_pin_, OUTPUT);
    pinMode(zf_dir_pin_, OUTPUT);
}

void VelocityJoint::update(unsigned long dt_ms)
{
    pulsesToRPM();
    setState(rpm_*RPM_TO_RAD_S);  // convert to rad/s
    GenericJoint::update(dt_ms);
}

void VelocityJoint::getState(float &state)
{
    pulsesToRPM();
    setState(rpm_*RPM_TO_RAD_S);
    state = state_;
}

void VelocityJoint::actuate()
{
    if (enabled_)
    {
        if (effort_ > 0) 
        { 
            digitalWrite(zf_dir_pin_, HIGH); 
            analogWrite(vr_speed_pin_, floorf(map(effort_,0.0f,100.0f,0,255)));
        }
        else
        { 
            digitalWrite(zf_dir_pin_, LOW);  
            analogWrite(vr_speed_pin_, floorf(map(-effort_,0.0f,100.0f,0,255)));
        }
    }
}

void VelocityJoint::stop()
{
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
        rpm_ = (float) pulses_ / 45.0f / ((float) interval_ * 1000000.0f) * 60.0f;
        pulses_ = 0;
        interval_ = 0;
    }

    interrupts();
}
