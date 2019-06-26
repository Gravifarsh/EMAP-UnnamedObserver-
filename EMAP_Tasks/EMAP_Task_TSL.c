/*
 * EMAP_Task_TSL.c
 *
 *  Created on: 25 июн. 2019 г.
 *      Author: developer
 */

#include <stm32f4xx_hal.h>
#include "diag/Trace.h"

#include "FreeRTOS.h"
#include "task.h"

#include "EMAPConfig.h"

#include "EMAP_Task_TSL.h"
#include "TSL2561.h"

void TSL_Init()
{
	HAL_StatusTypeDef error = 0;

	tsl2561.addr = TSL2561_ADDR_FLOAT;
	tsl2561.gain = TSL2561_GAIN_16X;
	tsl2561.intg = TSL2561_INT_100MS;
	tsl2561.type = TSL2561_TYPE_T;

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
	(HAL_I2C_Init(tsl2561.hi2c));

	uint8_t dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_CONTROL;
	PROCESS_ERROR( tsl2561_writeReg(&tsl2561, dummy, CONTROL_REG_VALUE_POWER_UP, 1) );
	dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_TIMING;
	PROCESS_ERROR( tsl2561_writeReg(&tsl2561, dummy, tsl2561.intg, 1) );
	dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_INTERRUPT;
	PROCESS_ERROR( tsl2561_writeReg(&tsl2561, dummy, 0x00, 1) );	//IRQ
	HAL_Delay(500);

/*
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
*/
end:
	trace_printf("LS: %d", error);
}

void TSL_Task()
{
	for(;;)
	{
		vTaskDelay(500 / portTICK_RATE_MS);
		tsl2561_readADC(&tsl2561, &data_TSL.ch0, &data_TSL.ch1);
		tsl2561_calcLux(&tsl2561, &data_TSL.lux, &data_TSL.ch0, &data_TSL.ch1);
		trace_printf("lux: %d", data_TSL.lux);
	}
}
