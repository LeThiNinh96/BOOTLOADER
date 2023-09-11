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
#include <stdint.h>


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
#define SPI_ADDRESS_BASE   0x40013000
void active_slave()
{
	//set PE3 LOW
	__HAL_RCC_GPIOE_CLK_ENABLE();
	uint32_t* GPIOE_ODR = (uint32_t*)(0x40021000 + 0x14);
	*GPIOE_ODR &= (0 << 3);
}

void inactive_slave()
{
	//set PE3 HIGH
	__HAL_RCC_GPIOE_CLK_ENABLE();
	uint32_t* GPIOE_ODR = (uint32_t*)(0x40021000 + 0x14);
	*GPIOE_ODR |= (1 << 3);
}

void SPI_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	//set GPIO PA5, 6, 7 alternate function mode
	uint32_t* GPIOA_MODER = (uint32_t*)(0x40020000 + 0x00);
	*GPIOA_MODER |= (0b10 << 10) | (0b10 << 12) | (0b10 << 14);  //set PA5, 6, 7 in alternate function
	uint32_t* GPIOA_AFRL = (uint32_t*)(0x40020000 + 0x20);
	*GPIOA_AFRL &= ~(0xfff << 20);
	*GPIOA_AFRL |= (0b0101 << 20) | (0b0101 << 24) | (0b0101 << 28);

	// set PE3 output mode
	uint32_t* GPIOE_MODER = (uint32_t*)(0x40021000 + 0x00);
	*GPIOE_MODER |= (0b01 << 6);

	inactive_slave();
	__HAL_RCC_SPI1_CLK_ENABLE();
	uint32_t* CR1 = (uint32_t*)(SPI_ADDRESS_BASE + 0x00);
	*CR1 |= (1<<2); // set stm32 as master
	*CR1 |= 0b100 << 3;
	*CR1 |= (1<<8) | (1<<9);
	*CR1 |= 1<<6;
}
 uint8_t SPI_READ(uint8_t address)
 {
	 uint32_t* SR = (uint32_t*)(SPI_ADDRESS_BASE + 0x08);
	 uint32_t* DR = (uint32_t*)(SPI_ADDRESS_BASE + 0x0C);

	 active_slave();

	 //while(((*SR >> 7) & 1) == 1);
	 while(((*SR >> 1) & 1) == 0);
	 *DR = address | (1 <<7);
	 while(((*SR >> 1) & 1) == 1);
	 while(((*SR >> 0) & 1) == 0);
	 while(((*SR >> 7) & 1) == 1);
	 uint8_t temp = *DR; // clear rx buffer
	 while(((*SR >> 1) & 1) == 0);
	 *DR = 0x00;   // send fake data to create clock
	 while(((*SR >> 1) & 1) == 1);
	 while(((*SR >> 0) & 1) == 0);
	 while(((*SR >> 7) & 1) == 1);
	 temp = *DR;

	 inactive_slave();

	 return temp;
 }

 void SPI_Write(uint8_t address, uint8_t data)
 {
	 uint32_t* SR = (uint32_t*)(SPI_ADDRESS_BASE + 0x08);
	 uint32_t* DR = (uint32_t*)(SPI_ADDRESS_BASE + 0x0C);

	 active_slave();

	 while(((*SR >> 7) & 1) == 1);
	 while(((*SR >> 1) & 1) == 0);
	 *DR = (1<<7)|address;
	 while(((*SR >> 1) & 1) == 1);
	 while(((*SR >> 0) & 1) == 0);
	 while(((*SR >> 7) & 1) == 1);
	 uint8_t temp = *DR; // clear rx buffer
	 while(((*SR >> 1) & 1) == 0);
	 *DR = data;   // send fake data to create clock
	 while(((*SR >> 1) & 1) == 1);
	 while(((*SR >> 0) & 1) == 0);
	 while(((*SR >> 7) & 1) == 1);
	 temp = *DR;

	 inactive_slave();

	 return temp;
 }
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
  SPI_Init();
  uint8_t gyro_id = SPI_READ(0x0f);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
