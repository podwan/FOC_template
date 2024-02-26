#ifndef __FOC_H
#define __FOC_H

#include "userMain.h"

#define PWM_GENERATE(p1, p2, p3) \
    {                            \
        TIM1->CCR1 = p1;         \
        TIM1->CCR2 = p2;         \
        TIM1->CCR3 = p3;         \
    }

/******************************************************************************/
// dq current structure
typedef struct
{
    float d;
    float q;
} DQCurrent_s;
// phase current structure
typedef struct
{
    float a;
    float b;
    float c;
} PhaseCurrent_s;
// dq voltage structs
typedef struct
{
    float d;
    float q;
} DQVoltage_s;



extern DQVoltage_s voltage;
extern DQCurrent_s current;



/******************************************************************************/
float getElectricalAngle(void);
/******************************************************************************/

void closeAngleLoop(float targetAngle);
void clarke(float iA, float iB, float iC, float *iAlpha, float *iBeta);
void closeSpeedLoop(float currentSpeed, float setSpeed, float theta, float iA, float iB, float iC, float frequence);
void setPhaseVoltage(float _voltageQ, float _voltageD, float _angleElectrical);
#endif
