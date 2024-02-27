#ifndef HELLOWORD_DYNAMIC_FW_KNOB_H
#define HELLOWORD_DYNAMIC_FW_KNOB_H

#include "userMain.h"
#include "bldcMotor.h"

typedef enum
{
    MODE_DISABLE = 0,
    MODE_INERTIA,
    MODE_ENCODER,
    MODE_SPRING,
    MODE_DAMPED,
    MODE_SPIN
} KnobMode;

void knobSimulatorInit(Motor *_motor);
void knobSimulatorTickTick();
void SetEnable(bool _en);
void ApplyZeroPos(float _angle = 0);
void SetMode(Mode_t _mode);
void SetLimitPos(float _min, float _max);
float GetPosition();
float GetVelocity();
int GetEncoderModePos();

#endif // HELLOWORD_DYNAMIC_FW_KNOB_H
