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

void Leds_init()
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	 uint32_t* GPIOD_MODER = (uint32_t*)(0x40020C00 + 0x00);
	 uint32_t* GPIOD_OTYPER = (uint32_t*)(0x40020C00 + 0x04);

	 /* set up PD 12 13 14 15 in OUTPUT */
	 *GPIOD_MODER &= ~(0xff << 24);
	 *GPIOD_MODER |= (0b01 << 24) | (0b01 << 26) | (0b01 << 28) | (0b01 << 30);

	 /* set up PD 12 13 14 15 in pussh-pull*/
	 *GPIOD_OTYPER &= ~(0xf << 12);

}
typedef enum
{
	LED_0, LED_1, LED_2, LED_3
}led_enum_t;

typedef enum
{
	LED_OFF, LED_ON
}led_state_t;

/**
 * @brief control led
 * @param
 * 		led_num uint8_t be 1(PD12),2,3 or 4
 * 		led_state uint8_t be 0 ->0v: low, 1->3v: high
 * @retval none
 */
void Leds_ctrl(led_enum_t led_enum, uint8_t led_state)
{
	uint32_t* GPIOD_ODR = (uint32_t*)(0x40020C00 + 0x14);
	if(led_state != 0)
	{
	*GPIOD_ODR |= (0b1 << (12 + led_enum)); /* set PD 12 to high --> LED on */
	}
	else
	{
	*GPIOD_ODR &= ~(0b1 << (12 + led_enum)); /* set PD 12 to high --> LED off */
	}
}

void goto_sleep()
{
	uint32_t* SCR = (uint32_t*)0xE000ED10;
	*SCR |= (1<<2);	//deepsleep
	uint32_t* PWR_CR = (uint32_t*)0x40007000;
	*PWR_CR |= (1<<1);
	__asm("WFI");	//standby
}

void watchdog_init()
{
	uint32_t* PR = (uint32_t*)0x40003004;
	uint32_t* KR = (uint32_t*)0x40003000;
	uint32_t* RLR = (uint32_t*)0x40003008;

	*KR = 0x5555;	//unclock PR and RLR register, write value to them
	*PR = 0b011;
	*RLR = 3000 - 1;
	*KR = 0xCCCC; //start wcatchdog


}
void feed_watchdog()
{
	uint32_t* KR = (uint32_t*)0x40003000;
	*KR = 0xAAAA;
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
  system_tick_init();
  Leds_init();
  watchdog_init();
//  Custom_delay(5000);
//  WD_init();
//  goto_sleep();

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
  int cnt = 0;
  int x = 0;
  int y = 10;
  while (1)
  {
    /* USER CODE END WHILE */
	  Leds_ctrl(LED_1, LED_ON);
	  Custom_delay(500);
	  Leds_ctrl(LED_1, LED_OFF);
	  Custom_delay(1000);
	  if(cnt++ > 5)
	  {
		  y = 10/x;
	  }
	  feed_watchdog();
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
