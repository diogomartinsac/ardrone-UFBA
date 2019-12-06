#ifndef PID_HPP 
#define PID_HPP

#include<iostream>

class PID
{
    public:
        double max, min, kp, ki, kd, setpoint, pv, error, error_prev, error_integral;
        PID(double max,double min,double kp,double ki,double kd);

        double calculate(double time, double setpoint, double pv);

};
#endif