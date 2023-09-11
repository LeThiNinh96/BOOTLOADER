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

uint32_t systick_cnt = 0;
volatile char rx_buffer[5852] = {0};
volatile char recv_data_done_flag = 0;
void SysTick_Handler()
{
	systick_cnt++;
}
void system_tick_init()
{
	uint32_t* CSR = (uint32_t* )0xe000e010;
	uint32_t* RVR = (uint32_t* )0xe000e014;
	*RVR = 15999;
	*CSR |= (1<<1)|(1<<0)|(1<<2);
}


void Custom_delay(uint32_t mSec)
{
	systick_cnt = 0;
	while(systick_cnt < mSec);
}

void TIM4_PWM_Init()
{
	//PWM T = 1s, do rong xung = 50%
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* GPIOD_MODER = (uint32_t*)(0x40020c00);
	*GPIOD_MODER |= (0b10 << 24);
	uint32_t* GPIOD_AFRH = (uint32_t*)(0x40020c24);
	*GPIOD_AFRH |= (2 << 16);

	__HAL_RCC_TIM4_CLK_ENABLE();
	uint32_t* TIM4_ARR = (uint32_t*)(0x4000082c);
	uint32_t* TIM4_PSC = (uint32_t*)(0x40000828);
	uint32_t* TIM4_CCR1 = (uint32_t*)(0x40000834);
	*TIM4_ARR = 100 - 1;
	*TIM4_PSC = 160 - 1;
	*TIM4_CCR1 = 25;

	uint32_t* TIM4_CR1 = (uint32_t*)(0x40000800);
	uint32_t* TIM4_CCMR1 = (uint32_t*)(0x40000818);
	uint32_t* TIM4_CCER = (uint32_t*)(0x40000820);

	*TIM4_CCMR1 &= ~(0b11<<0);  // set mode in output compare
	*TIM4_CCMR1 |= (0b110 << 4); // set CC1 mode in PWM1

	*TIM4_CCER |= (1<<0); // enable channel 1

	*TIM4_CR1 |= 1; // enable counter
}

void Modify_PWM(uint8_t value)
{
	uint32_t* CCR1 = (uint32_t*)(0x40000834);
	*CCR1 = value;

}
void TIM5_CAPTURE_Init()
{
	//set PA0 inalternate function AF02 (TIM5_CH1 )
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(0x40020000);
	*GPIOA_MODER &= ~(0b11 << 0);
	*GPIOA_MODER |= (0b10 << 0);
	uint32_t* GPIOA_AFRL = (uint32_t*)(0x40020020);
	*GPIOA_AFRL |= (2 << 0);

	__HAL_RCC_TIM5_CLK_ENABLE();
	// set counter
	uint32_t* TIM5_ARR = (uint32_t*)(0x40000c2c);
	uint32_t* TIM5_PSC = (uint32_t*)(0x40000c28);
	*TIM5_ARR = 0xffff;
	*TIM5_PSC = 16-1; // 1cnt -> 1us

	uint32_t* TIM5_CCMR1 = (uint32_t*)(0x40000c18);
	uint32_t* TIM5_CCER = (uint32_t*)(0x40000c20);
	// setup capture channel 1
	*TIM5_CCMR1 |= (0b01<<0); // set CH1 map TI1
	*TIM5_CCER &= ~((1<<1) | (1<<3));  //rising for channel 1

	// setup capture channel 2
	*TIM5_CCMR1 |= (0b10<<8); // set CH2 map TI1
	*TIM5_CCER &= ~(1<<7);  //falling for channel 2
	*TIM5_CCER |= (1<<5);

	//reset counter to 0 when rising
	uint32_t* TIM5_SMCR = (uint32_t*)(0x40000c08);
	*TIM5_SMCR |= 0b100; //set reset mode
	*TIM5_SMCR |= (0b101 << 4);

	// enable channel 1, channel 2, counter
	*TIM5_CCER |= (1<<0) | (1<<4);
	uint32_t* TIM5_CR1 = (uint32_t*)(0x40000c00);
	*TIM5_CR1 |= 1;
}

int Get_freq()
{
	uint32_t* TIM5_CCR1 = (uint32_t*)(0x40000c34);
	return *TIM5_CCR1;
}
int Get_pulse_width()
{
	uint32_t* TIM5_CCR2 = (uint32_t*)(0x40000c38);
	return *TIM5_CCR2;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int freq = 0;
int pulse_width = 0;
int main()
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  system_tick_init();
  TIM4_PWM_Init();
  Modify_PWM(50);
  TIM5_CAPTURE_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  freq = Get_freq();
	  pulse_width = Get_pulse_width();
	  Custom_delay(100);
    /* USER CODE BEGIN 3 */
  }
  return 0;
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
