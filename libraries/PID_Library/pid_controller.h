/**
* C++ PID Library
* Purpose: A general purpose C++ PID library
* @version 1.0 03/31/2016
* @author Kevin Huo
*/

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <Arduino.h>

class PID{
  public:
    PID(double kp_val, double ki_val, double kd_val, double min, double max);
    ~PID(){};
    double calculateOutput(double set_point, double current_point);
    void setOutputLimits(double min, double max);
    void setTunings(double kp_val, double ki_val, double kd_val);
    double getKp();
    double getKi();
    double getKd();

  private:
    double checkLimits(double output);
    unsigned long getDeltaTime(unsigned long current_time, unsigned long previous_time);
    double kp = 0.0, 
           ki = 0.0, 
           kd = 0.0, 
           min_output = 0.0,
           max_output = 0.0,
           previous_output = 0.0, 
           previous_error = 0.0, 
           previous_time = 0.0;
           
};

#endif