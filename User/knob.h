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

void knobSimulatorInit(BldcMotor *_motor);
void knobSimulatorTick(BldcMotor *motor);
void SetEnable(bool _en);
void ApplyZeroPos(float _angle);
void SetMode(KnobMode _mode);
void SetLimitPos(float _min, float _max);

int GetEncoderModePos(void);

#endif // HELLOWORD_DYNAMIC_FW_KNOB_H
