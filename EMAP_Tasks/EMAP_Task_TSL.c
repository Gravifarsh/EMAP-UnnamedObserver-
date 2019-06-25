/*
 * EMAP_Task_TSL.c
 *
 *  Created on: 25 июн. 2019 г.
 *      Author: developer
 */

#include <stm32f4xx_hal.h>
#include "diag/Trace.h"

#include "EMAP_Task_TSL.h"

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
