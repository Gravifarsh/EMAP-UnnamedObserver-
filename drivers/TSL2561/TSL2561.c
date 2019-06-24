/*
 * TSL2561.c
 *
 *  Created on: 1 июн. 2019 г.
 *      Author: developer
 */

#include <stm32f4xx_hal.h>
#include "diag/Trace.h"

#include "TSL2561.h"

I2C_HandleTypeDef	i2c_tsl2561;
tsl2561_t			tsl2561;

#define PROCESS_ERROR(x) if (0 != (error = (x))) { goto end; }

int tsl2561_readReg(I2C_HandleTypeDef * hi2c, uint8_t regAddr, uint8_t * buffer, uint8_t size)
{
	return HAL_I2C_Mem_Read(hi2c, TSL2561_ADDR_FLOAT, regAddr, I2C_MEMADD_SIZE_8BIT, buffer, size, 0xFF);
}

int tsl2561_writeReg(I2C_HandleTypeDef * hi2c, uint8_t regAddr, uint8_t buffer, uint8_t size)
{
	int error = 0;
	uint8_t regData = 0x00;
	PROCESS_ERROR(tsl2561_readReg(hi2c, regAddr, &regData, 1));

	uint8_t regData_new = (regData | buffer);
	return HAL_I2C_Mem_Write(hi2c, TSL2561_ADDR_FLOAT, regAddr, I2C_MEMADD_SIZE_8BIT, &regData_new, size, 0xFF);

end:
	return error;
}

int TSL_Init()
{
	int error = 0;

	i2c_tsl2561.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2c_tsl2561.Init.ClockSpeed = 200000;
	i2c_tsl2561.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c_tsl2561.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	i2c_tsl2561.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c_tsl2561.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	i2c_tsl2561.Init.OwnAddress1 = 0x00;
	i2c_tsl2561.Init.OwnAddress2 = 0x00;

	i2c_tsl2561.Instance = I2C1;
	i2c_tsl2561.Mode = HAL_I2C_MODE_MASTER;

	tsl2561.hi2c = &i2c_tsl2561;
	PROCESS_ERROR(HAL_I2C_Init(tsl2561.hi2c));
	//HAL_Delay(400);

	uint8_t dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_CONTROL;
	uint8_t buffer = 0x03;
	PROCESS_ERROR( tsl2561_writeReg(&i2c_tsl2561, dummy, buffer, 1) );
	dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_TIMING;
	buffer = 0b00000010;
	PROCESS_ERROR( tsl2561_writeReg(&i2c_tsl2561, dummy, buffer, 1) );
	dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_INTERRUPT;
	buffer = 0b00000000;
	PROCESS_ERROR( tsl2561_writeReg(&i2c_tsl2561, dummy, buffer, 1) );
	HAL_Delay(500);

	uint8_t buffer0[2] = { 0 };
	dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_CHAN0_LOW;
	PROCESS_ERROR( tsl2561_readReg(&i2c_tsl2561, dummy, &buffer0[1], 1) );

	dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_CHAN0_HIGH;
	PROCESS_ERROR( tsl2561_readReg(&i2c_tsl2561, dummy, &buffer0[0], 1) );
	trace_printf("LS CH0: 0x%d", buffer0[0] * 256 + buffer0[1]);

	uint8_t buffer1[2] = { 0 };
	dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_CHAN1_LOW;
	PROCESS_ERROR( tsl2561_readReg(&i2c_tsl2561, dummy, &buffer1[1], 1) );

	dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_CHAN1_HIGH;
	PROCESS_ERROR( tsl2561_readReg(&i2c_tsl2561, dummy, &buffer1[0], 1) );
	trace_printf("LS CH1: 0x%d", buffer1[0] * 256 + buffer1[1]);

end:
	trace_printf("LS: %d", error);
	return error;
}
