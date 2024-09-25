#ifndef _PID_SOURCE_C_
#define _PID_SOURCE_C_

#include "pid.h"

#include <math.h>
#include <stdlib.h>

struct PIDImpl {
  double _dt;
  double _max;
  double _min;
  double _Kp;
  double _Kd;
  double _Ki;
  double _pre_error;
  double _integral;
};

bool pid_init(pid_ref* pid, double dt, double max, double min, double Kp, double Kd, double Ki) {
  pid_ref tmp = (pid_ref)malloc(sizeof(struct PIDImpl));
  if (tmp == NULL) {
    return false;
  }
  tmp->_dt = dt;
  tmp->_max = max;
  tmp->_min = min;
  tmp->_Kp = Kp;
  tmp->_Kd = Kd;
  tmp->_Ki = Ki;
  tmp->_pre_error = 0;
  tmp->_integral = 0;
  *pid = tmp;
  return true;
}

double pid_calculate(pid_ref pid, double setpoint, double pv) {
  // Calculate error
  double error = setpoint - pv;

  // Proportional term
  double Pout = pid->_Kp * error;

  // Integral term
  pid->_integral += error * pid->_dt;
  double Iout = pid->_Ki * pid->_integral;

  // Derivative term
  double derivative = (error - pid->_pre_error) / pid->_dt;
  double Dout = pid->_Kd * derivative;

  // Calculate total output
  double output = Pout + Iout + Dout;

  // Restrict to max/min
  if (output > pid->_max)
    output = pid->_max;
  else if (output < pid->_min)
    output = pid->_min;

  // Save error to previous error
  pid->_pre_error = error;

  return output;
}

void pid_destroy(pid_ref pid) { free(pid); }

#endif
