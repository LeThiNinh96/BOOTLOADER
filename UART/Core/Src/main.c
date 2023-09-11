/*
 * main.c
 *
 *  Created on: Mar 14, 2023
 *      Author: NhanIMIC
 */
#include "main.h"
#include <string.h>
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


void UART1_Init()
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


void DMA_Init()
{
	__HAL_RCC_DMA2_CLK_ENABLE();
	uint32_t* S2CR = (uint32_t*)(0x40026440);
	*S2CR &= ~1;

	uint32_t* S2PAR = (uint32_t*)(0x40026448);
	*S2PAR = 0x40011004;

	uint32_t* S2M0AR = (uint32_t*)(0x4002644c);
	*S2M0AR = (uint32_t)rx_buffer;

	uint32_t* S2NDTR = (uint32_t*)(0x40026444);
	*S2NDTR = sizeof(rx_buffer);


	*S2CR = (0b100 << 25) | (1 << 10) | (1 << 8) | (1 << 4)| 1;
	//NVIC enable interrupt for DMA2_Stream2
	uint32_t* ISER1 = (uint32_t*)(0xE000e104);
	*ISER1 |= (1 << 26);
}

void DMA2_Stream2_IRQHandler()
{
	uint32_t* DMA_LIFCR = (uint32_t*)(0x40026400 + 0x08);
	*DMA_LIFCR |= (1 << 21);
	recv_data_done_flag = 1;
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

void UART1_Log(char* msg)
{
	int msg_len = strlen(msg);
	for(int i = 0; i < msg_len; i++)
	{
		UART1_Send(msg[i]);
	}
}
#define FLASH_ADDRESS_BASE 0x40023C00
__attribute__((section(".FuncInRam"))) int Erase(int sector_num)
{
	if((sector_num <0)||(sector_num > 7))
		return -1;
	//1. Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register
	uint32_t* SR = (uint32_t*)(FLASH_ADDRESS_BASE + 0x0C);
	while(((*SR >> 16)&1) == 1);

	//2. Set the SER bit and select the sector out of the 7 sectors (STM32F411xC/E) in the main memory block you wish to erase (SNB) in the FLASH_CR register
	uint32_t* CR = (uint32_t*)(FLASH_ADDRESS_BASE + 0x10);
	//After reset, write is not allowed in the Flash control register (FLASH_CR) to protect the Flash memory against possible
	if(((*CR >> 31) & 1) == 1)
	{
		uint32_t* KEYR = (uint32_t*)(FLASH_ADDRESS_BASE + 0x04);
		*KEYR = 0x45670123;
		*KEYR = 0xCDEF89AB;
	}
	*CR |= (1<<1);
	*CR |= (sector_num<<3);
	//3. Set the STRT bit in the FLASH_CR register
	*CR |= 1<<16;
	//4. Wait for the BSY bit to be cleared
	while(((*SR >> 16)&1) == 1);
	*CR &= ~(1<<1);

	return 0;
}

__attribute__((section(".FuncInRam"))) void Program(char* address, char* data, int data_size)
{
	//1. Check that no main Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register.
	uint32_t* SR = (uint32_t*)(FLASH_ADDRESS_BASE + 0x0C);
	while(((*SR >> 16)&1) == 1);
	//2. Set the PG bit in the FLASH_CR register
	uint32_t* CR = (uint32_t*)(FLASH_ADDRESS_BASE + 0x10);
	if(((*CR >> 31) & 1) == 1)
	{
		uint32_t* KEYR = (uint32_t*)(FLASH_ADDRESS_BASE + 0x04);
		*KEYR = 0x45670123;
		*KEYR = 0xCDEF89AB;
	}
	*CR |= 1<<0;
	//3. Perform the data write operation(s) to the desired memory address (inside main memory block or OTP area):
	for(int i = 0; i < data_size; i++)
	{
		address[i] = data[i];
	}
	//4. Wait for the BSY bit to be cleared.
	while(((*SR >> 16)&1) == 1);
	*CR &= ~(1<<0);
}

__attribute__((section(".FuncInRam"))) void reset_system()
{

}
__attribute__((section(".FuncInRam"))) void update_firmware()
{
	Erase(0);
	Program((char*)0x08000000, (char*)rx_buffer, sizeof(rx_buffer));
	reset_system();
}
int main()
{
	system_tick_init();
	UART1_Init();
	DMA_Init();
	UART1_Log("hello\r\n");

	while(1)
	{
		if (recv_data_done_flag == 1)
		{
			update_firmware();
		}
	}
	return 0;
}
