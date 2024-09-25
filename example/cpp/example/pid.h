#ifndef _PID_H_
#define _PID_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PIDImpl *pid_ref;

/**
 * 
 * Kp -  proportional gain
 * Ki -  Integral gain
 * Kd -  derivative gain
 * dt -  loop interval time
 * max - maximum value of manipulated variable
 * min - minimum value of manipulated variable
 */
bool pid_init(pid_ref* pid, double dt, double max, double min, double Kp, double Kd, double Ki);
/// Returns the manipulated variable given a setpoint and current process value
double pid_calculate(pid_ref pid, double setpoint, double pv);
void pid_destroy(pid_ref pid);

#ifdef __cplusplus
}
#endif

#endif
