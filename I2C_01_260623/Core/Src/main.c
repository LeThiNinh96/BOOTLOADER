/*
 * Custom_I2C.c
 *
 *  Created on: Jun 26, 2023
 *      Author: HP
 */

#include "main.h"
#include <string.h>
#include "Custom_I2C.h"
#include "math.h"

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

#define OUTX_L_M	0x68
#define OUTX_H_M	0x69
#define OUTY_L_M	0x6A
#define OUTY_H_M	0x6B
#define OUTZ_L_M	0x6C
#define OUTZ_H_M	0x6D
#define LIS2MDL_MAG_LSB 1.5
#define LIS2MDL_MILLIGAUSS_TO_MICROTESLA 0.1
uint16_t x_axis, y_axis, z_axis;
float heading;
int main()
{
	system_tick_init();
	I2C1_Init();
	const uint8_t accel_addr = 0b0011001;
	const uint8_t magne_addr = 0b0011110;
	uint8_t accel_id = I2C1_Read(accel_addr, 0x0f);
	uint8_t magne_id = I2C1_Read(magne_addr, 0x4f);
	I2C1_Write(magne_addr, 0x60, 0x00);
	uint8_t x_l, x_h, y_l, y_h, z_l, z_h;
	const float Pi = 3.14159;

	while(1)
	{
		x_l = I2C1_Read(magne_addr, OUTX_L_M);
		x_h = I2C1_Read(magne_addr, OUTX_H_M);
		y_l = I2C1_Read(magne_addr, OUTY_L_M);
		y_h = I2C1_Read(magne_addr, OUTY_H_M);
		z_l = I2C1_Read(magne_addr, OUTZ_L_M);
		z_h = I2C1_Read(magne_addr, OUTZ_H_M);
		x_axis = (x_h << 8) | x_l;
		y_axis = (y_h << 8) | y_l;
		z_axis = (z_h << 8) | z_l;

		magnetic_x = x_axis * LIS2MDL_MAG_LSB * LIS2MDL_MILLIGAUSS_TO_MICROTESLA;
		magnetic_y = y_axis * LIS2MDL_MAG_LSB * LIS2MDL_MILLIGAUSS_TO_MICROTESLA;
		heading = (atan2(magnetic_y,magnetic_x) * 180) / Pi;

		Custom_delay(500);
	}
	return 0;
}
