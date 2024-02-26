#include "userMain.h"
#include "bldcMotor.h"
#include "comm.h"
#include "led.h"
#include "key.h"
#include "app.h"

osThreadId_t ctrlLoopTaskHandle;
osThreadId_t commTaskHandle;
osThreadId_t appTaskHandle;
osThreadId_t keyTaskHandle;

void userMain()
{
    const osThreadAttr_t controlLoopTask_attributes = {
        .name = "ControlLoopTask",
        .stack_size = 256 * 10,
        .priority = (osPriority_t)osPriorityRealtime, // robot control thread is critical, should be the highest
    };
    ctrlLoopTaskHandle = osThreadNew(ThreadCtrlLoop, NULL, &controlLoopTask_attributes);

    const osThreadAttr_t commTask_attributes = {
        .name = "commTask",
        .stack_size = 256 * 5,
        .priority = (osPriority_t)osPriorityNormal,
    };
    commTaskHandle = osThreadNew(communicationLoop, NULL, &commTask_attributes);

    const osThreadAttr_t appTask_attributes = {
        .name = "appTask",
        .stack_size = 256 * 2,
        .priority = (osPriority_t)osPriorityNormal,
    };
    appTaskHandle = osThreadNew(appRunningLoop, NULL, &appTask_attributes);

    const osThreadAttr_t keyTask_attributes = {
        .name = "keyTask",
        .stack_size = 256 * 2,
        .priority = (osPriority_t)osPriorityNormal,
    };
    keyTaskHandle = osThreadNew(keyScanLoop, NULL, &keyTask_attributes);

    HAL_TIM_Base_Start_IT(&htim6);
}

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if (htim->Instance == TIM6)
//     {
//         BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//         // Wake & invoke thread IMMEDIATELY.
//         vTaskNotifyGiveFromISR(ctrlLoopTaskHandle, &xHigherPriorityTaskWoken);
//         portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//     }
// }
