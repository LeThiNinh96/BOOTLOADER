
#include "main.h"
#include <string.h>
void tim1_init_1sec()
{
	//enable clock for TIM1
	__HAL_RCC_TIM1_CLK_ENABLE();
	//set CNT = 0
	uint32_t* CNT = (uint32_t*)(0x40010000 + 0x24);
	*CNT = 0;
	//set auto-reload equal 1000-1
	uint32_t* TIM1_ARR = (uint32_t*)(0x40010000 + 0x2C);
	*TIM1_ARR = 5000-1;
	//set prescaler equal 16000-1
	uint32_t* TIM1_PSC = (uint32_t*)(0x40010000 + 0x28);
	*TIM1_PSC = 16000-1;
	//enable update event interrupt
	uint32_t* DIER = (uint32_t*)(0x4001000c);
	*DIER |= 1<<0;
	uint32_t* ISER0 = (uint32_t*)(0xE000e100);
	*ISER0 |= (1<<25);
	//start timer/counter
	uint32_t* CR1 = (uint32_t*)(0x40010000);
	*CR1 |= (1<<0) | (1<<2);
}

void TIM1_UP_TIM10_IRQHandler();
{
	__asm("NOP");
	uint32_t* SR = (uint32_t*)(0x40010010);
	*SR &= ~(1<<0);

}

void PWM_Led_Green()
{
	//PWM T = 1s, độ rộng xung = 50%
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* GPIOD_MODER = (uint32_t*)(0x)
}
