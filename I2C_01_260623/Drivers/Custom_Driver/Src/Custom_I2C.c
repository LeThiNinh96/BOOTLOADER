/*
 * Custom_I2C.c
 *
 *  Created on: Jun 26, 2023
 *      Author: HP
 */
#include "main.h"

void I2C1_Init()
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/*
		set PB6 as alternate function 4 (I2C1_SCL)
		PB9 as alternate function 4 (I2C1_SDA)
	*/
	uint32_t* GPIOB_MODER = (uint32_t*)(0x40020400 + 0x00);
	*GPIOB_MODER |= (0b10 << 12) | (0b10 << 18);
	uint32_t* GPIOB_AFRL = (uint32_t*)(0x40020420);
	*GPIOB_AFRL |= (0b0100 << 24);
	uint32_t* GPIOB_AFRH = (uint32_t*)(0x40020424);
	*GPIOB_AFRH |= (0b0100 << 4);

	//enable clock for I2C1
	__HAL_RCC_I2C1_CLK_ENABLE();

	/*set STM32 in master mode
	 master mode is auto switch when generate Start bit
	 */

	//set f for SCL (clock)
	uint32_t* I2C1_CR1 = (uint32_t*)(0x40005400);
	uint32_t* I2C1_CR2 = (uint32_t*)(0x40005404);
	uint32_t* I2C1_CCR = (uint32_t*)(0x4000541c);
	*I2C1_CR2 |= 16<<0;
	*I2C1_CCR |= 80<<0;

	//enable I2C1
	*I2C1_CR1 &= ~(0b1<<0);
	*I2C1_CR1 |= (0b1<<0);
}
uint8_t I2C1_Read(uint8_t sensor_addr, uint8_t reg_addr)
{
	uint32_t* I2C1_CR1 = (uint32_t*)(0x40005400);
	uint32_t* I2C1_DR = (uint32_t*)(0x40005410);
	uint32_t* I2C1_SR1 = (uint32_t*)(0x40005414);
	uint32_t* I2C1_SR2 = (uint32_t*)(0x40005418);

	while (((*I2C1_SR2 >> 1) & 1) == 1);   //check bit BUSY
	//generate start bit
	*I2C1_CR1 |= (1<<8);
	while (((*I2C1_SR1 >> 0) & 1) == 0);
	//send 7 bit slave address + WRITE bit (0)
	*I2C1_DR = (sensor_addr << 1) | 0;
	while(((*I2C1_SR1 >> 1) & 1) == 0);
	uint32_t temp = *I2C1_SR2;
	(void)temp;
	//check ACK
	while(((*I2C1_SR1 >> 10) & 1) == 1);
	//send data (what's data to read)
	*I2C1_DR = reg_addr;
	while(((*I2C1_SR1 >> 2) & 1) == 0);
	//check ACK
	while(((*I2C1_SR1 >> 10) & 1) == 1);
	//generate again start bit
	*I2C1_CR1 |= (1<<8);
	while (((*I2C1_SR1 >> 0) & 1) == 0);
	//send 7 bit slave address + READ bit (1)
	*I2C1_DR = (sensor_addr << 1) | 1;
	while (((*I2C1_SR1 >> 1) & 1) == 0);
	temp = *I2C1_SR2;
	//check ACK
	while(((*I2C1_SR1 >> 10) & 1) == 1);
	//read data from Slave
	while(((*I2C1_SR1 >> 6) & 1) == 0);
	uint8_t adta_recv = *I2C1_DR;
	//generate stop bit
	*I2C1_CR1 |= (1<<9);
	return adta_recv;
}
void I2C1_Write(uint8_t sensor_addr, uint8_t reg_addr, uint8_t value)
{
	uint32_t* I2C1_CR1 = (uint32_t*)(0x40005400);
	uint32_t* I2C1_DR = (uint32_t*)(0x40005410);
	uint32_t* I2C1_SR1 = (uint32_t*)(0x40005414);
	uint32_t* I2C1_SR2 = (uint32_t*)(0x40005418);

	while (((*I2C1_SR2 >> 1) & 1) == 1);   //check bit BUSY
	//generate start bit
	*I2C1_CR1 |= (1<<8);
	while (((*I2C1_SR1 >> 0) & 1) == 0);
	//send 7 bit slave address + WRITE bit (0)
	*I2C1_DR = (sensor_addr << 1) | 0;
	while(((*I2C1_SR1 >> 1) & 1) == 0);
	uint32_t temp = *I2C1_SR2;
	(void)temp;
	//check slave send ACK for master
	while(((*I2C1_SR1 >> 10) & 1) == 1);
	//send 8bit data (data type)
	*I2C1_DR = reg_addr;
	while (((*I2C1_SR1 >> 2) & 1) == 0);
	//check slave send ACK for master
	while(((*I2C1_SR1 >> 10) & 1) == 1);
	//send 8 bit data (write data)
	*I2C1_DR = value;
	while(((*I2C1_SR1 >> 2) & 1) == 0);
	//check slave send ACK for master
	while(((*I2C1_SR1 >> 10) & 1) == 1);
	//generate stop bit
	*I2C1_CR1 |= (1<<9);
}
