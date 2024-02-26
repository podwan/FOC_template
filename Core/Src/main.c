/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "foc.h"
#include "bldcMotor.h"
#include "userMain.h"
#include "comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define PHASE_SHIFT_ANGLE (float)(220.0f / 360.0f * 2.0f * _PI)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern DMA_HandleTypeDef hdma_usart3_tx;

uint8_t Uart1_Rx_Cnt = 0;
float Vbus, Ia, Ib, Ic;
uint16_t IA_Offset, IB_Offset, IC_Offset;
uint16_t adc1_in1, adc1_in2, adc1_in3, Vpoten, adc_vbus;
uint8_t ADC_offset = 0;
float alpha = 0.3;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);
  // HAL_UART_Receive_IT(&huart3, rxBuffer, RX_CMD_LEN);
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
  __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC);
  // HAL_ADCEx_InjectedStart_IT(&hadc1);
  // HAL_ADCEx_InjectedStart(&hadc2);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/******************************************************************************/

/******************************************************************************/

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//   /* Prevent unused argument(s) compilation warning */
//   UNUSED(GPIO_Pin);
//   if (Button2_Pin == GPIO_Pin)
//   {
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
//   }
//   if (Button3_Pin == GPIO_Pin)
//   {
//     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
//     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
//     HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
//     HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
//     HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
//   }
//   /* NOTE: This function should not be modified, when the callback is needed,
//            the HAL_GPIO_EXTI_Callback could be implemented in the user file
//    */
// }
//  MotorMode motorMode;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) // 10kHz ADC
{
  static uint8_t cnt;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  if (hadc == &hadc1)
  {
    if (ADC_offset == 0)
    {
      cnt++;
      adc1_in1 = hadc1.Instance->JDR1;
      adc1_in2 = hadc2.Instance->JDR1;
      adc1_in3 = hadc1.Instance->JDR2;
      IA_Offset += adc1_in1;
      IB_Offset += adc1_in2;
      IC_Offset += adc1_in3;
      if (cnt >= 10)
      {
        ADC_offset = 1;
        IA_Offset = IA_Offset / 10;
        IB_Offset = IB_Offset / 10;
        IC_Offset = IC_Offset / 10;
      }
      // motorMode = OPEN_LOOP;
    }
    else
    {
      adc1_in1 = hadc1.Instance->JDR1;
      adc1_in3 = hadc1.Instance->JDR2;
      adc1_in2 = hadc2.Instance->JDR1;
      Ia = (adc1_in1 - IA_Offset) * 0.02197f;
      Ib = (adc1_in2 - IB_Offset) * 0.02197f;
      Ic = (adc1_in3 - IC_Offset) * 0.02197f; // 0.02197265625

      // openSpeedLoop(3, 60);
      // closeAngleLoop(target);

#if SEND_RCC_DATA
      // temp[0] = Ia;
      // temp[1] = Ib;
      // memcpy(tempData, (uint8_t *)&temp, sizeof(temp));
      // HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tempData, 6 * 4);
#endif
    }
  }
}
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   /* Prevent unused argument(s) compilation warning */
//   UNUSED(huart);
//   if (Uart1_Rx_Cnt >= USART_BUFFER_SIZE - 1)
//   {
//     Uart1_Rx_Cnt = 0;
//     memset(rxBuffer, 0x00, sizeof(rxBuffer));
//     HAL_UART_Transmit(&huart3, (uint8_t *)"too long\n", 10, 0xFFFF);
//   }
//   else
//   {
//     rxBuffer[Uart1_Rx_Cnt++] = aRxBuffer;
//     if (aRxBuffer == '\n')
//     {
//       Uart1_Rx_Cnt = 0;
//       setUartRecvDone();
//     }
//     //		Uart1_Rx_Cnt = 0;
//     //		memset(rxBuffer,0x00,sizeof(rxBuffer));
//   }
//   HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);
//   /* NOTE: This function should not be modified, when the callback is needed,
//            the HAL_UART_TxHalfCpltCallback can be implemented in the user file.
//    */
// }

int fputc(int ch, FILE *f)
{
  while ((USART3->ISR & 0X40) == 0)
    ;
  USART3->TDR = (uint8_t)ch;
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM6)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Wake & invoke thread IMMEDIATELY.
    vTaskNotifyGiveFromISR(ctrlLoopTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
