#include "pid_controller.h"

/**
* Contructor function for the PID object. Sets the kp, ki, kd
* values as well as the min/max output limits.
* @param kp_val The P tuning parameter.
* @param ki_val The I tuning parameter.
* @param kd_val The D tuning parameter.
* @param min The minimum output limit.
* @param max The maximum output limit.
*/
PID::PID(double kp_val, double ki_val, double kd_val, double min, double max)
{
  setTunings(kp_val, ki_val, kd_val);
  setOutputLimits(min, max);
}

/**
* calculateOutput calculates the error based on goal position and current position.
* Using the change in error and the change in time since the last iteration,
* it approximates the integral and derivative under the error curve, and then
* calculates the output based on the tuning values. The function will then save 
* the current values to be used for the next iteration.
* This function uses the ptime microsec_clock::universal_time() object from the
* boost posix_time library.
* @param set_point The goal position
* @param current_point The current position
* @returns The output values.
* Outputs a value between the min and max output limit.
*/
double PID::calculateOutput(double set_point, double current_point)
{
  unsigned long current_time = millis();
  double error = current_point - set_point;
  unsigned long delta_time = getDeltaTime(current_time, previous_time);

  //Calculate the area under the error curve (Riemann Sum)
  double error_integral = error * delta_time;

  //Calculate the derivative (change in error / change in time)
  double delta_error = (error - previous_error) / delta_time;

  //Calculate PID output based on tunings
  double output = kp * error + ki * error_integral + kd * delta_error;

  //Make sure output is within the min/max limits
  //output = checkLimits(output);

  //Save current values to be used in the calculations for next iteration
  previous_output = output;
  previous_error = error;
  previous_time = current_time;

  return output;
}

/**
* setOutputLimits sets the limitations to the output value.
* @param min The minimum output required.
* @param max The maximum output required.
*/
void PID::setOutputLimits(double min, double max)
{
  min_output = min;
  max_output = max;
}

/**
* setTunings sets the tuning parameters.
* @param kp_val The P tuning parameter.
* @param ki_val The I tuning parameter.
* @param kd_val The D tuning parameter.
*/
void PID::setTunings(double kp_val, double ki_val, double kd_val)
{
  kp = kp_val;
  ki = ki_val;
  kd = kd_val;
}

/**
* checkLimits takes in the calculated outputs and makes sure that it
* is within the min/max output limit. If the proposed output is greater
* than max, it will set output as max. If the proposed output is less
* than min, it will set output as min. Otherwise output stays the same.
* @param output The raw output value before the check.
* @returns An output value within the limited bounds.
*/
double PID::checkLimits(double output)
{
  if(output >= max_output)
  {
    return max_output;
  }
  else if(output <= min_output)
  {
    return min_output;
  }
  
  return output;
}

unsigned long PID::getDeltaTime(unsigned long current_time, unsigned long previous_time)
{
  if(current_time < previous_time)
  {
    return current_time + (~((unsigned long) 0) - previous_time);
  }

  return current_time - previous_time;
}

/**
* getKp
* @returns The kp tuning parameter.
*/
double PID::getKp()
{
  return kp;
}

/**
* getKi
* @returns The ki tuning parameter.
*/
double PID::getKi()
{
  return ki;
}

/**
* getKd
* @returns The kd tuning parameter.
*/
double PID::getKd()
{
  return kd;
}