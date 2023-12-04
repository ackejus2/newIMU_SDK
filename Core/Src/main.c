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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "REG.h"
#include "stm32l4xx_hal.h"  // Include HAL driver for STM32L4
#include <math.h>   // Include Math library for calculations
#include <string.h>   // Include for strlen
#include <stdio.h>    // Include for snprintf
#include "wit_c_sdk.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define sensor addresses and commands
//#define IICADDR (0x50 << 1) // IMU address shifted for HAL library
//#define WIT_TIME 0x50
//#define WIT_ACC 0x51
//#define WIT_QUATER 0x59
#define ACC_SC (16.0 / 32768.0)
#define QUATER_SC (1.0 / 32768.0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float accel[3] = {0, 0, 0};
float vel_i[3] = {0, 0, 0};
float vel_f[3] = {0, 0, 0};
float pos[3] = {0, 0, 0};
float quat[4] = {0, 0, 0, 0};
float euler[3] = {0, 0, 0};
float time_i = 0;
float time_f;
float deltaTime;
float time_elapsed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

HAL_StatusTypeDef UART_SendString(UART_HandleTypeDef *huart, char *str) {
    uint16_t len = strlen(str);
    return HAL_UART_Transmit(huart, (uint8_t *)str, len, HAL_MAX_DELAY);
}

void readTime(I2C_HandleTypeDef *hi2c, uint8_t *buffer) {
    HAL_I2C_Mem_Read(&hi2c1, IICADDR, WIT_TIME, I2C_MEMADD_SIZE_8BIT, buffer, 8, HAL_MAX_DELAY);
    // Process time data as needed
}

void readAcceleration(void) {
    uint8_t data[3];
    data[0] = WitReadReg(AX, WIT_ACC);
    data[1] = WitReadReg(AY, WIT_ACC);
    data[2] = WitReadReg(AZ, WIT_ACC);
    uint8_t ahigh0 = data[0] >> 8 & 0xFF;
    uint8_t alow0  = data[0] << 8 & 0xFF;
    uint8_t ahigh1 = data[1] >> 8 & 0xFF;
    uint8_t alow1  = data[1] << 8 & 0xFF;
    uint8_t ahigh2 = data[2] >> 8 & 0xFF;
    uint8_t alow2  = data[2] << 8 & 0xFF;
    accel[0] = ((ahigh0 << 8) | alow0); //* ACC_SC;
    accel[1] = ((ahigh1 << 8) | alow1); //* ACC_SC;
    accel[2] = ((ahigh2 << 8) | alow2); //* ACC_SC;
}

void readQuaternion(void) {
    uint8_t data[4];
    data[0] = WitReadReg(q0, 1);
    data[1] = WitReadReg(q1, 1);
    data[2] = WitReadReg(q2, 1);
    data[3] = WitReadReg(q3, 1);
    uint8_t qhigh0 = data[0] >> 8 & 0xFF;
    uint8_t qlow0  = data[0] << 8 & 0xFF;
    uint8_t qhigh1 = data[1] >> 8 & 0xFF;
    uint8_t qlow1  = data[1] << 8 & 0xFF;
    uint8_t qhigh2 = data[2] >> 8 & 0xFF;
    uint8_t qlow2  = data[2] << 8 & 0xFF;
    uint8_t qhigh3 = data[3] >> 8 & 0xFF;
    uint8_t qlow3  = data[3] << 8 & 0xFF;
    quat[0] = ((qhigh0 << 8) | qlow0) * QUATER_SC;
    quat[1] = ((qhigh1 << 8) | qlow1) * QUATER_SC;
    quat[2] = ((qhigh2 << 8) | qlow2) * QUATER_SC;
    quat[3] = ((qhigh3 << 8) | qlow3) * QUATER_SC;
}

void quaternionToEuler(const float *quat, float *euler) {
    // Convert quaternion to Euler angles
    euler[0] = atan2(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]), 1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));
    euler[1] = asin(2.0f * (quat[0] * quat[2] - quat[3] * quat[1]));
    euler[2] = atan2(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]), 1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]));
}

void computeRelativePosition(const float *accel, float deltaTime, float *pos) {
    // Simple integration of acceleration to compute position change
    for (int i = 0; i < 3; ++i) {
        vel_f[i] = vel_i[i] + accel[i] * deltaTime;
        pos[i] = (vel_i[i] * deltaTime) + ((1/2) * (accel[i] * deltaTime * deltaTime));
        vel_i[i] = vel_f[i];
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float accel[3], quat[4], euler[3], pos[3] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // Initialize IMU for I2C
  	 WitInit(3, IICADDR);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  const char *openingStatement = "UART Trial\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)openingStatement, strlen(openingStatement), 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  time_f = WitReadReg(MS, WIT_TIME);
	  time_elapsed += time_f;
	  deltaTime = time_f - time_i;
	  readAcceleration();
	  readQuaternion();
	  quaternionToEuler(quat, euler);
	  computeRelativePosition(accel, deltaTime, pos);
	  time_i = time_f;
//	  // Read time data
//	  readTime(&hi2c1, timeData);
//
//	  // Read acceleration data
//	  readAcceleration(&hi2c1, accel);
//
//	  // Read quaternion data
//	  readQuaternion(&hi2c1, quat);
//
//	  // Convert quaternion to Euler angles
//	  quaternionToEuler(quat, euler);
//
//	  // Compute relative position (deltaTime should be updated appropriately)
//	  float deltaTime = 1.0; // Example: 1 second
//	  computeRelativePosition(accel, deltaTime, pos);
//
	  // Use timeData, euler, and position as needed
	  char positionString[128]; // Adjust the size as needed
	  snprintf(positionString, sizeof(positionString), "Position: X: %f, Y: %f, Z: %f\r\n", pos[0], pos[1], pos[2]);

	  HAL_UART_Transmit(&huart2, (uint8_t*)positionString, strlen(positionString), 1000);

	  // ...

	  HAL_Delay(1000); // Example delay, adjust as needed
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
