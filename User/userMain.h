#ifndef __USER_MAIN_H
#define __USER_MAIN_H

#define PWM_PERIOD 8000.0f // PWM period * 2
#define U_DC 12.0f
#define uQ_MAX (U_DC / _SQRT3)
// #define uQ_MAX U_DC
#define I_MAX 3         // 最大电流
#define FREQUENCE 10000 // FOC compute frequency which is current sampling frequency

// 设置使用的编码器为1，不使用的为0
#define AS5600 1 // 编码器类型，只能选一
#define AS5047P 0
#define TLE5012B 0
#define MA730 0
#define MT6701 0
#define SEND_RCC_DATA 0
#define CALI_PID 1

typedef unsigned int uint;
typedef unsigned char uchar;

typedef enum
{
    false,
    true
} bool;

typedef union
{
    unsigned char byte;
    struct
    {
        unsigned char b0 : 1;
        unsigned char b1 : 1;
        unsigned char b2 : 1;
        unsigned char b3 : 1;
        unsigned char b4 : 1;
        unsigned char b5 : 1;
        unsigned char b6 : 1;
        unsigned char b7 : 1;
    } bits;
} Byte;

typedef enum
{
    NONE_KEY,
    USER1_SHORT,
    USER2_SHORT,
    USER3_SHORT,
} KeyState;

// below are common heads which are not created by user but should be included in every user's file
#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include <math.h>
#include "usart.h"

#define SHOW_RCC_DATA 0
void userMain(void);
void printLog(void);
extern osThreadId_t ctrlLoopTaskHandle;
#endif
