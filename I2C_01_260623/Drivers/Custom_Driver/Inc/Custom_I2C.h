/*
 * Custom_I2C.h
 *
 *  Created on: Jun 26, 2023
 *      Author: HP
 */

#ifndef CUSTOM_DRIVER_INC_CUSTOM_I2C_H_
#define CUSTOM_DRIVER_INC_CUSTOM_I2C_H_
#include <stdint.h>

void I2C1_Init();
uint8_t I2C1_Read(uint8_t sensor_addr, uint8_t reg_addr);
void I2C1_Write(uint8_t sensor_addr, uint8_t reg_addr, uint8_t value);


#endif /* CUSTOM_DRIVER_INC_CUSTOM_I2C_H_ */
