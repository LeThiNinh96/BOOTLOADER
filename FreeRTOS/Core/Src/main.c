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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
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
/* Definitions for task1 */
osThreadId_t task1Handle;
const osThreadAttr_t task1_attributes = {
  .name = "task1",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task2 */
osThreadId_t task2Handle;
const osThreadAttr_t task2_attributes = {
  .name = "task2",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task3 */
osThreadId_t task3Handle;
const osThreadAttr_t task3_attributes = {
  .name = "task3",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task4 */
osThreadId_t task4Handle;
const osThreadAttr_t task4_attributes = {
  .name = "task4",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for temp_queue */
osMessageQueueId_t temp_queueHandle;
const osMessageQueueAttr_t temp_queue_attributes = {
  .name = "temp_queue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void func1(void *argument);
void func2(void *argument);
void func3(void *argument);
void func4(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
void UART1_init()
{
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	uint32_t* MODER = (uint32_t*)(0x40020400);
	*MODER |= (0b10 << 12) | (0b10 << 14);		//set PB6 (U1Tx), PB7(U1Rx)

	uint32_t* AFRL = (uint32_t*)(0x40020420);
	*AFRL  |= (0b0111 << 24) | (0b0111 << 28);

	uint32_t* BRR = (uint32_t*)(0x40011008);
	*BRR = (104<<4) | 3;

	uint32_t* CR3 = (uint32_t*)(0x40011014);
	*CR3 |= 1 << 6;							//enable DMA for receiver

	uint32_t* CR1 = (uint32_t*)(0x4001100c);
	*CR1 |= (1<< 3)|(1<<2)|(1<<13);
}
void UART1_Send(char data)
{
	uint32_t* SR = (uint32_t*)(0x40011000);
	uint32_t* DR = (uint32_t*)(0x40011004);
	while(((*SR >> 7) & 1) != 1);
	*DR	= data;
	while(((*SR >> 6) & 1) != 1);
	*SR &= ~(1 << 6);
}
void UART1_Log(char* format, ...)
{
	va_list arg;
    va_start(arg, format);
    char send_buff[128] = {0};
    vsprintf(send_buff, format, arg);
    va_end(arg);
	int msg_len = strlen(send_buff);
	for(int i = 0; i < msg_len; i++)
	{
		UART1_Send(send_buff[i]);
	}
}

void adc_init()
{
	//set PC2 in analog mode

	__HAL_RCC_ADC1_CLK_ENABLE();
	uint32_t* SMR1 = (uint32_t*)(0x4001200c);
	uint32_t* CR2 = (uint32_t*)(0x40012008);

	uint32_t* CCR = (uint32_t*)0x40012304;

	*SMR1 |= 0b111<< 18;
	uint32_t* JSQR = (uint32_t*)(0x40012038);
	*JSQR &= ~(0b11 << 20); //select 1 convertion
	*JSQR |= (16<<15); //select channel 16 (temp sensor) for JSQ4
	*CCR |= 1<<23; // enable temp sensor

	*CR2 |= 1<<0; //enable ADC

}

float adc_measure()
{
	uint32_t* CR2 = (uint32_t*)0x40012008;
	*CR2 |= (1<<22); //start injected channel to convert adc
	uint32_t* SR = (uint32_t*)0x40012000;
	while(((*SR >> 2) & 1) == 0);
	*SR &= ~(1<<2);
	uint32_t* JDR1 = (uint32_t*)0x4001203c;

	return 3.0*(*JDR1)/4095.0;
}
float temperature;
float temp_measure()
{
	float vsense = adc_measure();
	const float v25 = 0.76;
	const float avg_slope = 2.5/1000;
	float temp = ((vsense - v25)/avg_slope) + 25;
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
  /* USER CODE BEGIN 2 */
  Leds_init();
  UART1_init();
  adc_init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of temp_queue */
  temp_queueHandle = osMessageQueueNew (16, sizeof(float), &temp_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task1 */
  task1Handle = osThreadNew(func1, NULL, &task1_attributes);

  /* creation of task2 */
  task2Handle = osThreadNew(func2, NULL, &task2_attributes);

  /* creation of task3 */
  task3Handle = osThreadNew(func3, NULL, &task3_attributes);

  /* creation of task4 */
  task4Handle = osThreadNew(func4, NULL, &task4_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

/* USER CODE BEGIN Header_func1 */
/**
  * @brief  Function implementing the task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_func1 */
void func1(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	Leds_ctrl(LED_1, LED_ON);
    osDelay(1000);
    Leds_ctrl(LED_1, LED_OFF);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_func2 */
/**
* @brief Function implementing the task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func2 */
void func2(void *argument)
{
  /* USER CODE BEGIN func2 */
  /* Infinite loop */
  for(;;)
  {
    Leds_ctrl(LED_2, LED_ON);
    osDelay(1500);
    Leds_ctrl(LED_2, LED_OFF);
	osDelay(1500);
  }
  /* USER CODE END func2 */
}

/* USER CODE BEGIN Header_func3 */
/**
* @brief Function implementing the task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func3 */
void func3(void *argument)
{
  /* USER CODE BEGIN func3 */
  /* Infinite loop */

  for(;;)
  {
	float temp = 0;
	UART1_Log("temperature: [");
	int msg_cnt = osMessageQueueGetCount(temp_queueHandle);
	for(int i = 0; i < msg_cnt; i++)
	{
		osMessageQueueGet(temp_queueHandle, &temp, 0, HAL_MAX_DELAY);
		UART1_Log("%.2f", temp);
	}
	osMessageQueueGet (temp_queueHandle, &temp, 0, HAL_MAX_DELAY);

	int msg_cnt1 = osMessageQueueGetCount(temp_queueHandle);
	UART1_Log("temperature % d - %d\r\n", msg_cnt, msg_cnt1);
    osDelay(1000);
  }
  /* USER CODE END func3 */
}

/* USER CODE BEGIN Header_func4 */
/**
* @brief Function implementing the task4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func4 */
void func4(void *argument)
{
  /* USER CODE BEGIN func4 */
  /* Infinite loop */
  for(;;)
  {
	float temp = temp_measure();
	osMessageQueuePut(temp_queueHandle, &temp, 0, HAL_MAX_DELAY);
    osDelay(100);
  }
  /* USER CODE END func4 */
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
