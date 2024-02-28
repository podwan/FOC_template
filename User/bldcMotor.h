#ifndef __BLDC_MOTOR_H
#define __BLDC_MOTOR_H

#include "userMain.h"
#include "lowPassFilter.h"
#include "pid.h"
#define M0_Disable HAL_TIM_Base_Stop(&htim1)
/******************************************************************************/

/******************************************************************************/
/**
 *  Motiron control type
 */
typedef enum
{
    TORQUE,   //!< Torque control
    VELOCITY, //!< Velocity motion control
    ANGLE,    //!< Position/angle motion control
    VELOCITY_OPEN_LOOP,
    ANGLE_OPEN_LOOP,
} ControlMode;

/**
 *  Motiron control type
 */
typedef enum
{
    Type_voltage,    //!< Torque control using voltage
    Type_dc_current, //!< Torque control using DC current (one current magnitude)
    Type_foc_current //!< torque control using dq currents
} TorqueControlType;

typedef struct
{
    ControlMode controlMode;
    float voltageUsedForSensorAlign;
    float velocityLimit;
    float currentLimit;
    float voltageLimit;
    LowPassFilter lpfCurrentQ;
    LowPassFilter lpfCurrentD;
    LowPassFilter lpfVelocity;
    LowPassFilter lpfAngle;
    PidController pidCurrentQ;
    PidController pidCurrentD;
    PidController pidVelocity;
    PidController pidAngle;
    float target;
} BldcMotor;

extern float target;
extern BldcMotor bldcMotor;
extern float voltagePowerSupply;
extern float shaftVelocity;
/******************************************************************************/
extern float shaftAngle; //!< current motor angle
extern float electricalAngle;
// extern float shaftVelocity;
extern float setPointCurrent;
extern float setPointVelocity;
extern float setPointAngle;

void motorInit(void);
void motorTick(BldcMotor *motor);
void ThreadCtrlLoop(void *argument);
void motorSetTorqueLimit(BldcMotor *motor, float _val);
#endif
