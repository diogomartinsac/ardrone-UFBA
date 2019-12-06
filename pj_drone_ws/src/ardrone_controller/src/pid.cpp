#include <ardrone_controller/pid.hpp>

PID::PID(double max,double min,double kp,double ki,double kd){
    this->max = max;
    this->min = min;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->error = 0;
    this->error_integral = 0;
    this->error_prev = 0;
}

double PID::calculate(double time, double setpoint, double pv){
    this->error = setpoint - pv;
    this->error_integral += this->error;

    double mv = kp*this->error + ki*(this->error_integral)*time + kd*(this->error - this->error_prev)/time;
    
    if (mv>max)
        mv = max;
    else if (mv<min)
        mv = min;
    
    this->error_prev = this->error;

    return mv;
}
