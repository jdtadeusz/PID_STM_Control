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
#define FILTER_SIZE 3
#define TUBE_LENGTH_MM 500.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t readings[FILTER_SIZE];
uint8_t read_index = 0;

PID_TypeDef hpid;

uint32_t current_dist_glob; // Zmienna globalna dla monitora
float setpoint_glob;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
int _write(int file, char *ptr, int len);
/* USER CODE BEGIN PFP */

// Filtr do eliminacji szumów czujnika - nieużywany
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

  if (VL53L0X_Init(&hi2c1) != HAL_OK) {
    Error_Handler(); 
  }
  
  // tryb ciągły
  VL53L0X_StartContinous(&hi2c1);

  // Dla Kp = 0.8 - oscylacje zaczynajace od -50 + 80, aż do -20 + 20
  PID_Init(&hpid, 0.8f, 0.007f, 0.05f, -100.0f, 200.0f);
  PID_SetSetpoint(&hpid, 190.0f); 
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  // 2050,5 - minimalna wartość żeby podnieść piłeczke z obciazeniem
  // 1265,0 - minimalna wartość żeby podnieść piłeczkę bez obciążenia
  
  uint32_t last_time = HAL_GetTick();
  uint32_t last_i2c_poll = HAL_GetTick(); // Zmienna dla przepustnicy

// Dynamiczna baza PWM (Kompensacja nieliniowości rury)
float Get_Dynamic_Feedforward(float setpoint) {
    float SP_low  = 240.0f;  float PWM_low  = 1290.0f;  // Dół rury
    float SP_high =  60.0f;  float PWM_high = 1305.0f;  // Góra rury

    if (setpoint >= SP_low) return PWM_low;
    if (setpoint <= SP_high) return PWM_high;

    float t = (setpoint - SP_low) / (SP_high - SP_low);
    return PWM_low + t * (PWM_high - PWM_low);
}
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);  
        HAL_Delay(10);         
    }

    if (HAL_GetTick() - last_i2c_poll >= 5) { 
        last_i2c_poll = HAL_GetTick();

        uint16_t raw_dist = VL53L0X_ReadContinuousFast(&hi2c1);

        // martwa strefa
        if (raw_dist > 0 && raw_dist <= 45) {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000); // Twarde odcięcie wiatru
            continue; 
        }

        if (raw_dist > 45 && raw_dist < 600) {
            
            uint32_t now = HAL_GetTick();
            float dt = (float)(now - last_time) / 1300.0f;
            last_time = now;
            if (dt <= 0.001f) dt = 0.001f;

            PID_SetDt(&hpid, dt);

            float dynamic_base = Get_Dynamic_Feedforward(hpid.setpoint);
      
            float pid_correction = PID_Compute(&hpid, (float)raw_dist);

            float final_pwm = dynamic_base + pid_correction;

            // Nasycenie sprzętowe
            if (final_pwm > 1600.0f) final_pwm = 1600.0f; 
            if (final_pwm < 1000.0f) final_pwm = 1000.0f; 

            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)final_pwm);
            printf("SP:%d, Dist:%d, Corr:%d, PWM:%d, dt:%d ms\r\n", 
                  (int)hpid.setpoint, (int)raw_dist, (int)pid_correction, (int)final_pwm, (int)(dt * 1000.0f));
        }
    }
  }
  
  /* USER CODE END WHILE */
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

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
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
