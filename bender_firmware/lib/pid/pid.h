#ifndef BENDER_FIRMWARE_PID_H
#define BENDER_FIRMWARE_PID_H

#define DT_SCALE 0.000001f // microsecond

#include <cmath>
#include "bender_utils.h"

class Pid
{
    public:
        
        Pid(float p=0.0, float i=0.0, float d=0.0, float iMin=0.0, float iMax=0.0);
        void  reset();
        void  getGains(float &p, float &i, float &d);
        void  setGains(float p, float i, float d);
        float computeCommand(float error, unsigned long dt_ms);
        float computeCommand(float error, float error_dot, unsigned long dt_ms);

    private:
        struct Gains {
            float p_gain_;
            float i_gain_;
            float d_gain_;

            Gains(float p, float i, float d) : p_gain_(p), i_gain_(i), d_gain_(d) {}
            Gains() : p_gain_(0.0), i_gain_(0.0), d_gain_(0.0) {}
        } gains;
        bool  valid_p_error_last_; /**< Is saved position state valid for derivative state calculation */
        float p_error_last_; /**< Save position state for derivative state calculation. */
        float p_error_; /**< Position error. */
        float i_error_; /**< Integral of position error. */
        float i_max_;
        float i_min_;
        bool antiwindup_;
        float d_error_; /**< Derivative of position error. */
        float cmd_;     /**< Command to send. */
}; // class Pid

#endif // BENDER_FIRMWARE_PID_H