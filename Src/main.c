/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "stm32f10x.h"

#ifdef USE_STM3210B_EVAL
 #include "stm3210b_eval.h"
 #include "stm3210b_eval_lcd.h"
 #define USE_BOARD
 #define USE_LED
#elif defined USE_STM3210E_EVAL
 #include "stm3210e_eval.h"
 #include "stm3210e_eval_lcd.h"
 #define USE_BOARD
 #define USE_LED
#elif defined USE_STM3210C_EVAL
 #include "stm3210c_eval.h"
 #include "stm3210c_eval_lcd.h"
 #include "stm3210c_eval_i2c_ee.h"
 #define USE_BOARD
 #define USE_LED
 #define USE_SEE
#elif defined USE_STM32100B_EVAL
 #include "stm32100b_eval.h"
 #include "stm32100b_eval_lcd.h"
 #define USE_BOARD
 #define USE_LED
#elif defined USE_STM32100E_EVAL
 #include "stm32100e_eval.h"
 #include "stm32100e_eval_lcd.h"
 #include "stm32100e_eval_i2c_ee.h"
 #define USE_BOARD
 #define USE_LED
 #define USE_SEE
#elif defined USE_STM32_DISCOVERY
 #include "STM32vldiscovery.h"
#elif defined USE_IAR_STM32F103ZE
 #include "board.h"
 #define USE_LED
#elif defined USE_KEIL_MCBSTM32
 #include "board.h"
 #define USE_LED
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t buffer[20];
   while (1)
   {	buffer[0]=0;
     /* USER CODE END WHILE */
   HAL_UART_Receive(&huart1,buffer,20,100);
            switch(buffer[0])
            {
                case 'w':
                    przod(); break;
                case 's':
                    tyl(); break;
                case 'd':
                    prawo(); break;
                case 'a':
                    lewo(); break;

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_TIM3_PARTIAL();

}

/* USER CODE BEGIN 4 */
void przod(void)
      {
          TIM3->CCR1 = 100;
          TIM2->CCR3 = 100;
          HAL_GPIO_WRITEPin(pster1_GPIO_Port, pster1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WRITEPin(pster2_GPIO_Port, pster2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WRITEPin(lster1_GPIO_Port, lster1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WRITEPin(lster2_GPIO_Port, lster2_Pin, GPIO_PIN_RESET);
      }

      void prawo(void)
      {
          TIM3->CCR1 = 500;
          TIM2->CCR3 = 100;
          HAL_GPIO_WRITEPin(pster1_GPIO_Port, pster1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WRITEPin(pster2_GPIO_Port, pster2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WRITEPin(lster1_GPIO_Port, lster1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WRITEPin(lster2_GPIO_Port, lster2_Pin, GPIO_PIN_RESET);
      }

      void lewo(void)
      {
          TIM3->CCR1 = 100;
          TIM2->CCR3 = 500;
          HAL_GPIO_WRITEPin(pster1_GPIO_Port, pster1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WRITEPin(pster2_GPIO_Port, pster2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WRITEPin(lster1_GPIO_Port, lster1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WRITEPin(lster2_GPIO_Port, lster2_Pin, GPIO_PIN_RESET);
      }

      void tyl(void)
      {
          TIM3->CCR1 = 100;
          TIM2->CCR3 = 100;
          HAL_GPIO_WRITEPin(pster1_GPIO_Port, pster1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WRITEPin(pster2_GPIO_Port, pster2_Pin, GPIO_PIN_SET);
          HAL_GPIO_WRITEPin(lster1_GPIO_Port, lster1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WRITEPin(lster2_GPIO_Port, lster2_Pin, GPIO_PIN_SET);
      }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
