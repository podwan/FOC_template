#ifndef __SENSOR_H
#define __SENSOR_H

#include "userMain.h"
#include "bldcMotor.h"
#define AUTO_CALI 1

/******************************************************************************/
#define AS5600_CPR 4096    // 12bit
#define AS5047P_CPR 16384  // 14bit
#define TLE5012B_CPR 32768 // 15bit
#define MA730_CPR 65536    // 14bit,左对齐,低两位补0,所以是65536
#define MT6701_CPR 65536   // 14bit,左对齐,低两位补0,所以是65536
/**
 *  Direction structure
 */
typedef enum
{
    CW = 1,     // clockwise
    CCW = -1,   // counter clockwise
    UNKNOWN = 0 // not yet known or invalid state
} Direction;

typedef enum
{
    NO_ERROR = 0,
    FAILED_TO_NOTICE_MOVEMENT,
    POLE_PAIR_MISMATCH
} Error_t;

extern long cpr;
extern float angle_prev;
extern Direction countDirection;
extern int polePairs;
extern float zero_electric_angle;
extern float sensor_offset;

/******************************************************************************/
float getElectricalAngle(void);
float getShaftVelocity(BldcMotor *motor);
/******************************************************************************/

/******************************************************************************/
void magneticSensorInit(float zero_electric_offset, Direction _sensor_direction, BldcMotor *motor);
float getAngle(void);
float getVelocity(void);
bool alignSensor(BldcMotor *motor);
float getShaftAngle(void);
int getRPM(void);
/******************************************************************************/
#endif
