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
    upper_limit_(upperLimit),
    lower_limit_(lowerLimit),
    target_(0.0),
    state_(0.0),
    effort_(0.0),
    effort_limit_(effortLimit) 
{
}

void GenericJoint::update(unsigned long dt_ms)
{
    float error;
    if      (target_ <= lower_limit_)  { error = state_ - lower_limit_; } 
    else if (target_ >= upper_limit_)  { error = state_ - upper_limit_; } 
    else                               { error = state_ - target_; }
    
    if (dt_ms <= 100)
    {
        effort_ = clamp(pid_controller_.computeCommand(error, dt_ms),
                        -effort_limit_, effort_limit_);
    }
}


/*********************************************************************
 * PositionJoint implementations
*********************************************************************/
PositionJoint::PositionJoint(int encAPin, int encBPin, int pwmPin, int dirPin,
                             float p, float i, float d) :
    GenericJoint(p, i, d, -M_PI_2, M_PI_2),
    enc_a_pin_(encAPin),
    enc_b_pin_(encBPin),
    pwm_pin_(pwmPin),
    dir_pin_(dirPin),
    encoder_(encAPin,encBPin) 
{ 
}

void PositionJoint::update(unsigned long dt_ms)
{
    setState(encoder_.read());
    double error;
    angles::shortest_angular_distance_with_limits(state_,
                                                  target_,
                                                  lower_limit_,
                                                  upper_limit_,
                                                  error);
    
    if (dt_ms <= 100)
    {
        effort_ = clamp(pid_controller_.computeCommand((float) error, dt_ms),
                        -effort_limit_, effort_limit_);
    }
}

void PositionJoint::actuate()
{
    if (effort_ > 0) { digitalWrite(dir_pin_, HIGH); }
    else             { digitalWrite(dir_pin_, LOW);  }
    analogWrite(pwm_pin_, effort_);
}

void PositionJoint::stop()
{
    effort_ = 0.0;
    analogWrite(pwm_pin_, 0.0);
}


/*********************************************************************
 * VelocityJoint implementations
*********************************************************************/
VelocityJoint::VelocityJoint(int vrPin, int zfPin, int interruptPin, 
                             float p, float i, float d, float velLimit) :
    GenericJoint(p, i, d, -velLimit, velLimit),
    vr_speed_pin_(vrPin),
    zf_dir_pin_(zfPin),
    tach_pin_(interruptPin) 
{
}

void VelocityJoint::actuate()
{
    if (effort_ > 0) { digitalWrite(zf_dir_pin_, HIGH); }
    else             { digitalWrite(zf_dir_pin_, LOW);  }
    analogWrite(vr_speed_pin_, effort_);
}

void VelocityJoint::stop()
{
    effort_ = 0.0;
    analogWrite(vr_speed_pin_, 0.0);
}
