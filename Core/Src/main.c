/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "crc.h"
#include "i2c.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_tim.h"
#include "stm32l4xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l0x.h"
#include <stdint.h>
#include <stdio.h>
#include "pid_controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILTER_SIZE 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t readings[FILTER_SIZE];
uint8_t read_index = 0;

PID_TypeDef hpid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Filtr do eliminacji szumów czujnika
uint16_t smooth_filter(uint16_t new_val) {
    readings[read_index] = new_val;
    read_index = (read_index + 1) % FILTER_SIZE;
    
    uint32_t sum = 0;
    for(int i = 0; i < FILTER_SIZE; i++) sum += readings[i];
    return (uint16_t)(sum / FILTER_SIZE);
}

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
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // HAL_StatusTypeDef status = VL53L0X_Init(&hi2c1);
  PID_Init(&hpid, 1.5f, 0.1f, 0.05f, 800.0f, 3199.0f);
  PID_SetSetpoint(&hpid, 250.0f); // 250mm
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  uint16_t distance = 0; 
  /* USER CODE END 2 */
  
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t sys_start = 0x01;
    uint8_t dev_status = 0;
    uint8_t data_raw[2] = {0, 0};
    char uart_buf[64];

    // Startu pomiaru
    HAL_I2C_Mem_Write(&hi2c1, VL53L0X_ADDR, 0x00, 1, &sys_start, 1, 100);

    // Polling
    uint32_t tickstart = HAL_GetTick();
    while (HAL_GetTick() - tickstart < 100) {
        HAL_I2C_Mem_Read(&hi2c1, VL53L0X_ADDR, 0x14, 1, &dev_status, 1, 10);
        if (dev_status & 0x01) break; 
    }

    // Sprawdzanie czy dane są gotowe
    if (dev_status & 0x01) {
    if (HAL_I2C_Mem_Read(&hi2c1, VL53L0X_ADDR, 0x1E, 1, data_raw, 2, 100) == HAL_OK) {
        uint16_t raw_dist = (uint16_t)((data_raw[0] << 8) | data_raw[1]);

        if (raw_dist > 20 && raw_dist < 2000) { 
            distance = smooth_filter(raw_dist);
        } else if (raw_dist == 8191) { // 8191 to błąd 

        }

        int len = sprintf(uart_buf, "Dist: %u | Raw: %u | Stat: 0x%02X\r\n", 
                          distance, raw_dist, dev_status);
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, len, 100);
        }
    
    uint32_t next_pwm = (uint32_t)PID_Compute(&hpid, (float)distance);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, next_pwm);

    // Czyszczenie przerwania
    uint8_t clear = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, VL53L0X_ADDR, 0x0B, 1, &clear, 1, 100);
    }

    // Częstotliwość pętli
    HAL_Delay(20); 

    /* USER CODE END WHILE */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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



/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
