#include"main.h"


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

void button_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(0x40020000 + 0x00);
	uint32_t* GPIOA_PUPDR = (uint32_t*)(0x40020000 + 0x0C);

	*GPIOA_MODER &= ~(0b11 << 0);
	*GPIOA_PUPDR &= ~(0b11 << 0);

}
char button_get_state()
{
	uint32_t* GPIOA_IDR = (uint32_t*)(0x40020000 + 0x10);
	char button_state = 0;
	button_state = (*GPIOA_IDR >> 0) & 1;
	return button_state;
}
uint32_t systick_cnt = 0;
void SysTick_Handler()
{
	systick_cnt++;
}
void system_tick_init()
{
	uint32_t* CSR = (uint32_t*)0xe000e010;
	uint32_t* RVR = (uint32_t*)0xe000e014;

	*RVR = 15999;
	*CSR |= (1<<1)|(1<<0)|(1<<2);
}

void Custom_delay(uint32_t mSec)
{
	systick_cnt = 0;
	while(systick_cnt < mSec);
}
int main()
{
	Leds_init();
	system_tick_init();
	while(1)
	{
		Leds_ctrl(LED_1, LED_ON);
		Custom_delay(1000);
		Leds_ctrl(LED_1, LED_OFF);
		Custom_delay(1000);
	}
	return 0;
}
