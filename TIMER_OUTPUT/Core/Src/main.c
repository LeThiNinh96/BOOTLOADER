
#include "main.h"
#include <stdint.h>

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

void PWM_Led_Green()
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
	*TIM4_PSC = 1600 - 1;
	*TIM4_CCR1 = 10 - 1;

	uint32_t* TIM4_CR1 = (uint32_t*)(0x40000800);
	uint32_t* TIM4_CCMR1 = (uint32_t*)(0x40000818);
	uint32_t* TIM4_CCER = (uint32_t*)(0x40000820);

	*TIM4_CCMR1 &= ~(0b11<<0);  // set mode in output compare
	*TIM4_CCMR1 |= (0b110 << 4); // set CC1 mode in PWM1

	*TIM4_CCER |= (1<<0); // enable channel 1

	*TIM4_CR1 |= 1; // enable counter
}

void PWM_Ctrl_Motor()
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
//công suất nằm trong khoảng [0:100]
void Motor_change_power(uint8_t power)
{
	uint32_t* TIM4_CCR1 = (uint32_t*)(0x40000834);
	*TIM4_CCR1 = power;
}
int input_capture()
{

}

int main()
{
	PWM_Ctrl_Motor();
	system_tick_init();
	int power = 0;
	double Kp = 1;
	double Ki = 1;
	double Kd = 1;
	double e = 0;
	double e_old = 0;
	double dao_ham, tich_phan = 0;
	double set_point = 0;
	double power = 0;
	while(1)
	{
		e = set_point - input_capture();
		dao_ham = (e - e_old)/(0.001);
		tich_phan += e*(0.001);
		power = Kp*e + Ki*tich_phan + Kd*dao_ham;
		if(power < 0)
			power = 0;
		else if (power > 100)
				power = 100;
		Motor_change_power(power);
		Custom_delay(1);

	}
	return 0;
}
