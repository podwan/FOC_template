#ifndef __LED_H
#define __LED_H

#include "userMain.h"

#define LED1_ON (HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET))
#define LED1_OFF (HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET))

#define LED2_ON (HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET))
#define LED2_OFF (HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET))

extern bool led1On, led2On;

void LED_drive(void);

#endif
