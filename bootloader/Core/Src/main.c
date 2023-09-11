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
void DMA_UART2_Rx_Init()
{
	__HAL_RCC_DMA1_CLK_ENABLE();
	// todo: Setup DMA1 stream 5 - chanel 4 for UARt2_Rx
	uint32_t* DMA_S5CR = (uint32_t*)(0x40026000 + 0x10 + 0x18*5);
	*DMA_S5CR |= (0b100 << 25); // select chanel 4 for stream 5
	*DMA_S5CR |= (0b1 << 10); // Set MINC

	// todo: Setup source address
	// 0x40004404 = 0x40004400 + 0x04 --> UART2_DR address
	uint32_t* DMA_S5PAR = (uint32_t*)(0x40026000 + 0x18 + 0x18*5);
	*DMA_S5PAR = 0x40004404;
	// todo: Setup distance address
	uint32_t* DMA_S5M0AR = (uint32_t*)(0x40026000 + 0x1C + 0x18*5);
	*DMA_S5M0AR = rx_buf;
	// todo: Setup size of data
	uint32_t* DMA_S5NDTR = (uint32_t*)(0x40026000 + 0x14 + 0x18*5);
	*DMA_S5NDTR = sizeof(rx_buf);

	// enable DMA1 stream 5
	*DMA_SxCR |= 1;
}
void UART_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(0x40020000 + 0x00);
	*GPIOA_MODER |= (0b10 << 4) | (0b10 << 6);  //set PA2, PA3 in alternate function
	uint32_t* GPIOA_AFRL = (uint32_t*)(0x40020000 + 0x20);
	*GPIOA_AFRL |= (0b0111 << 8) | (0b0111 << 12); // set PA2 as U2Tx, PA3 as U2Rx
	// todo: enable clock for UARTx
	__HAL_RCC_USART2_CLK_ENABLE(); //enable 16Mhz clock for UART2

	// todo: set baudrate 9600bps
    /*
    baudrate = fck/(16*UARTDIV)
    9600 = 16Mhz/(16*UARTDIV)
    --> UARTDIV = 104.167
    mantissa = 104
    franction = 0.167 * 16 = 2.67 ~ 3

    */
    uint32_t* USART_BRR = (uint32_t*)(0x40004400 + 0x08);
    *USART_BRR = (104<<4) | (3<<0);
    // todo: set frame data: 8bit data and no parity
    uint32_t* USART_CR1 = (uint32_t*)(0x40004400 + 0x0C);
    *USART_CR1 &= ~(1<<12);    // 1 start bit, 8 Data bits, n Stop bits
    *USART_CR1 &= ~(1<<10);    // Parity control disabled
#if 0
    //enable interrupt for RNXE
    *USART_CR1 |= (1<<5);
    uint32_t* ISER1 = (uint32_t*)0xe000e104;
    *ISER1 |= (1 << 6);
#else
    //enable DA for Rx
    uint32_t* USART_CR3 = (uint32_t*)(0x40004400 + 0x14);
    *USART_CR3 |= (1 << 6);
    DMA_UART2_Rx_Init();
#endif
    // todo: enable UART
    *USART_CR1 |= (1<<2) | (1<<3) | (1<<13); // enable transmitter, receiver and uart

}

void UART_Send_Byte(char _data)
{
	// todo: wait _DR empty
	uint32_t* USART_SR = (uint32_t*)(0x40004400 + 0x00);
	while(((*USART_SR >> 7) & 1) == 0);

    // todo: write "_data" to DR (data register)
	uint32_t* USART_DR = (uint32_t*)(0x40004400 + 0x04);
    *USART_DR = _data;
    // todo: wait transmission complete -> read bit 6 in SR

    while(((*USART_SR >> 6) & 1) == 0);
    *USART_SR &= ~(1<<6);
}

uint8_t UART_Recv_Byte()
{
	uint32_t* USART_SR = (uint32_t*)(0x40004400 + 0x00);
	uint32_t* USART_DR = (uint32_t*)(0x40004400 + 0x04);
	uint8_t data_recv = 0;

	while(((*USART_SR >> 5) & 1) == 0); // read data from DR
	data_recv = *USART_DR; // Câu lệnh đọc giá trị thanh ghi _DR
	return data_recv;
}

int rx_index = 0;
char rx_buf[32] = {0};
void USART2_IRQHandler()
{
	uint32_t* USART_DR = (uint32_t*)(0x40004400 + 0x04);
	rx_buf[rx_index] = *USART_DR;
	if(rx_index++ >= sizeof(rx_buf))
	rx_index = 0;
}

int main()
{
	button_init();
	Leds_init();
	system_tick_init();
	UART_init();

//	UART_Send_Byte('x');
	char msg[] = "stm32 hello laptop\r\n";

	for(int i = 0; i < sizeof(msg) - 1; i++)
	{
		UART_Send_Byte(msg[i]);
	}

	while(1)
	{

	}
	return 0;
}
