#include "pid.h"

Pid::Pid(float p, float i, float d, float iMin, float iMax) :
         gains(p,i,d),
         valid_p_error_last_(false),
         p_error_last_(0.0),
         p_error_(p),
         i_error_(i),
         i_max_(iMax),
         i_min_(iMin),
         antiwindup_(true),
         d_error_(d),
         cmd_(0.0)
{
}


void Pid::reset()
{
    valid_p_error_last_ = false;
    p_error_last_ = 0.0;
    p_error_ = 0.0;
    i_error_ = 0.0;
    d_error_ = 0.0;
    cmd_ = 0.0;
}


void Pid::getGains(float &p, float &i, float &d)
{
    p = gains.p_gain_;
    i = gains.i_gain_;
    d = gains.d_gain_;
}


void Pid::setGains(float p, float i, float d)
{
    gains.p_gain_ = p;
    gains.i_gain_ = i;
    gains.d_gain_ = d;
}


float Pid::computeCommand(float error, unsigned long dt_ms)
{
    if (error == 0.0 || dt_ms == 0 || std::isnan(error) || std::isinf(error)) {
        return 0.0;
    }

    float error_dot = d_error_;

    // Calculate the derivative error
    if (dt_ms > 100) {
        valid_p_error_last_ = false;
        error_dot = 0.0;
    }
    if (dt_ms > 0)
    {
        if (valid_p_error_last_) {
            error_dot = (error - p_error_last_) / (dt_ms * DT_SCALE);
        }
        p_error_last_ = error;
        valid_p_error_last_ = true;
    } 

    return computeCommand(error, error_dot, dt_ms);
}


float Pid::computeCommand(float error, float error_dot, unsigned long dt_ms)
{
    float p_term, d_term, i_term;
    p_error_ = error; // this is error = target - state
    d_error_ = error_dot;

    if (dt_ms == 0 || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
    {
        return 0;
    }

    // Calculate proportional contribution to command
    p_term = gains.p_gain_ * p_error_;

    // Calculate the integral of the position error
    i_error_ += (dt_ms * DT_SCALE) * p_error_;

    // Calculate integral contribution to command
    i_term = gains.i_gain_ * i_error_;
    if(antiwindup_)
    {
        // Limit i_term so that the limit is meaningful in the output
        i_term = clamp(i_term, i_min_, i_max_);
    }

    // Calculate derivative contribution to command
    d_term = gains.d_gain_ * d_error_;

    // Compute the command
    cmd_ = p_term + i_term + d_term;

    return cmd_;
}
