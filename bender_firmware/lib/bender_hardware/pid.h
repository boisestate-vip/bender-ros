#ifndef BENDER_FIRMWARE_PID_H
#define BENDER_FIRMWARE_PID_H

#include <ros.h>

class Pid
{
    public:
        struct Gains {
            double p_gain;
            double i_gain;
            double d_gain;

            Gains(double p, double i, double d) : p_gain(p), i_gain(i), d_gain(d) {}
            Gains() : p_gain(0.0), i_gain(0.0), d_gain(0.0) {}
        };
        Pid(double p = 0.0, double i = 0.0, double d = 0.0);

        void   reset();
        void   getGains(double &p, double &i, double &d);
        void   setGains(const Gains &gains);
        double computeCommand(double error, ros::Duration dt);
        double computeCommand(double error, double error_dot, ros::Duration dt);
        void   setCurrentCmd(double cmd);
        double getCurrentCmd();

    private:
        bool valid_p_error_last_; /**< Is saved position state valid for derivative state calculation */
        double p_error_last_; /**< Save position state for derivative state calculation. */
        double p_error_; /**< Position error. */
        double i_error_; /**< Integral of position error. */
        double d_error_; /**< Derivative of position error. */
        double cmd_;     /**< Command to send. */
}; // class PositionJoint