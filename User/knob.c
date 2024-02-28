#include <cmath>
#include <cstdio>
#include "knob.h"
#include "math_utils.h"
#include "bldcMotor.h"
#include "sensor.h"

BldcMotor *motor;
KnobMode mode = MODE_DISABLE;
float zeroPosition = 0;
float limitPositionMax = 5.1;
float limitPositionMin = 3.3;
int encoderDivides = 12;
int encoderPosition = 0;

float lastAngle;
float lastVelocity;

static float knobGetPosition(BldcMotor *motor);
static float knobGetVelocity(BldcMotor *motor); ///);

void knobSimulatorInit(BldcMotor *motor)
{

    motor->controlMode = TORQUE;
    motor->voltageLimit = 1.5;
    motor->velocityLimit = 100;

    pidInit(&motor->pidVelocity, 0.1, 0, 0, 1000, -motor->voltageLimit, motor->voltageLimit);
    pidInit(&motor->pidAngle, 80, 0, 0.7, 0, -motor->velocityLimit, motor->velocityLimit);
}

void knobSimulatorSetMode(BldcMotor *motor, KnobMode _mode)
{
    mode = _mode;

    lastAngle = knobGetPosition(motor);
    lastVelocity = knobGetVelocity(motor);

    switch (mode)
    {
    case MODE_DISABLE:

        break;
    case MODE_INERTIA:
    {

        motorSetTorqueLimit(motor, 0.5);
        motor->controlMode = VELOCITY;
        motor->pidVelocity.P = 0.1;
        motor->pidVelocity.I = 0.0;
        motor->pidVelocity.D = 0.0;
        motor->pidAngle.P = 20;
        motor->pidAngle.I = 0;
        motor->pidAngle.D = 0.7;
        motor->target = 0;
    }
    break;
    case MODE_ENCODER:
    {

        motorSetTorqueLimit(motor, 1.5);
        motor->controlMode = ANGLE;
        motor->pidVelocity.P = 0.1;
        motor->pidVelocity.I = 0.0;
        motor->pidVelocity.D = 0.0;
        motor->pidAngle.P = 100;
        motor->pidAngle.I = 0;
        motor->pidAngle.D = 3.5;
        motor->target = 4.2;
        lastAngle = 4.2;
    }
    break;
    case MODE_SPRING:
    {

        motorSetTorqueLimit(motor, 1.5);
        motor->controlMode = ANGLE;
        motor->pidVelocity.P = 0.1;
        motor->pidVelocity.I = 0.0;
        motor->pidVelocity.D = 0.0;
        motor->pidAngle.P = 100;
        motor->pidAngle.I = 0;
        motor->pidAngle.D = 3.5;
        motor->target = 4.2;
    }
    break;
    case MODE_DAMPED:
    {

        motorSetTorqueLimit(motor, 1.5);
        motor->controlMode = VELOCITY;
        motor->pidVelocity.P = 0.1;
        motor->pidVelocity.I = 0.0;
        motor->pidVelocity.D = 0.0;
        motor->target = 0;
    }
    break;
    case MODE_SPIN:
    {

        motorSetTorqueLimit(motor, 1.5);
        motor->controlMode = VELOCITY;
        motor->pidVelocity.P = 0.3;
        motor->pidVelocity.I = 0.0;
        motor->pidVelocity.D = 0.0;
        motor->target = 20;
    }
    break;
    }
}

void knobSimulatorTick(BldcMotor *motor)
{
    switch (mode)
    {
    case MODE_INERTIA:
    {
        float v = knobGetVelocity(motor);
        if (v > 1 || v < -1)
        {
            if (abs(v - lastVelocity) > 0.3)
                motor->target = v;
        }
        else
        {
            motor->target = 0;
        }
        lastVelocity = v;
    }
    break;
    case MODE_ENCODER:
    {
        float a = knobGetPosition(motor);
        if (a - lastAngle > _PI / (float)encoderDivides)
        {
            motor->target += _2PI / (float)encoderDivides;
            lastAngle = motor->target;
            encoderPosition++;
        }
        else if (a - lastAngle < -_PI / (float)encoderDivides)
        {
            motor->target -= _2PI / (float)encoderDivides;
            lastAngle = motor->target;
            encoderPosition--;
        }
    }
    break;
    case MODE_DAMPED:
        if (limitPositionMax != 0 && limitPositionMin != 0)
        {
            float a = knobGetPosition(motor);
            if (a > limitPositionMax)
            {
                motor->controlMode = ANGLE;
                motor->target = limitPositionMax;
            }
            else if (a < limitPositionMin)
            {
                motor->controlMode = ANGLE;
                motor->target = limitPositionMin;
            }
            else
            {
                motor->controlMode = VELOCITY;
                motor->target = 0;
            }
        }
        break;
    case MODE_DISABLE:
    case MODE_SPRING:
    case MODE_SPIN:
        break;
    }

    motorTick(motor);
}

void knobSimulatorSetLimitPos(float _min, float _max)
{
    limitPositionMin = _min;
    limitPositionMax = _max;
}

// void knobSimulatorApplyZeroPos(float _angle)
// {
//     if (_angle != 0)
//         zeroPosition = _angle;
//     else
//         zeroPosition = motor->GetEstimateAngle();
// }

float knobGetPosition(BldcMotor *motor)
{
    return getShaftAngle(motor) - zeroPosition;
}

float knobGetVelocity(BldcMotor *motor)
{
    return getShaftVelocity(motor);
}

// int knobSimulatorGetEncoderModePos()
// {
//     return std::lround(knobGetPosition() / (_2PI / (float)encoderDivides));
// }

// void KnobSimulator::SetEnable(bool _en)
// {
//     SetPowerMotor(_en);
//     motor->SetEnable(_en);
// }
