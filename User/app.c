
#include "app.h"
#include "led.h"
#include "key.h"
#include "bldcMotor.h"

static DevState devState;
static KeyState keyState;
static uchar flashCnt;

static void standingBy()
{
    led1On = 1;
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    if (keyState == USER1_SHORT)
    {
        WORK_INIT;
    }
}

static void working(void)
{
    if (flashCnt < 5)
        led2On = 1;

    if (keyState == USER1_SHORT)
    {
        STANDBY_INIT;
    }
    else if (keyState == USER2_SHORT)
    {
        if (bldcMotor.controlMode == VELOCITY)
        {
            if (target == 0)
            {
                target = 100;
            }
            else if (target == 100)
            {
                target == -100;
            }
            else if (target == -100)
            {
                target = 100;
            }
        }
    }
}

void appRunning()
{
    getKeyState(&keyState);

    if (++flashCnt >= 10)
        flashCnt = 0;

    led1On = 0;
    led2On = 0;

    switch (devState)
    {
    case STANDBY:
        standingBy();
        break;

    case WORK:
        working();
        break;
    }

    LED_drive();
}

void appRunningLoop(void *argument)
{
    devState = STANDBY;
    for (;;)
    {
        appRunning();
        osDelay(100);
    }
}
