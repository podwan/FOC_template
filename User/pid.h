#ifndef __PID_H
#define __PID_H

#include "userMain.h"
/******************************************************************************/
typedef struct
{
    float P;                      //!< Proportional gain
    float I;                      //!< Integral gain
    float D;                      //!< Derivative gain
    float output_ramp;            //!< Maximum speed of change of the output value
    float integral_prev;          //!< last integral component value
    float output_prev;            //!< last pid output value
    float error_prev;             //!< last tracking error value
    float outMax;                 //!< Maximum output value
    float outMin;                 //!< Mininum output value
    unsigned long timestamp_prev; //!< Last execution timestamp
} PidController; 
/******************************************************************************/

/******************************************************************************/
void pidInit(PidController *pid, float kp, float ki, float kd, float output_ramp, float outMax, float outMin);
float PID_operator(PidController *pid, float error);
/******************************************************************************/
#endif
