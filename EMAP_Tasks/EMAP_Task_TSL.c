/*
 * EMAP_Task_TSL.c
 *
 *  Created on: 25 июн. 2019 г.
 *      Author: developer
 */

#include "EMAP_Task_TSL.h"
#include "EMAPConfig.h"

void TSL_Init()
{
	HAL_StatusTypeDef error = 0;

	tsl2581.addr = TSL2581_ADDR_FLOAT;
	tsl2581.gain = TSL2581_GAIN_16X;
	tsl2581.intg = TSL2581_INT_399MS;
	tsl2581.type = TSL2581_TYPE_T;

	i2c_tsl2581.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2c_tsl2581.Init.ClockSpeed = 50000;
	i2c_tsl2581.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c_tsl2581.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	i2c_tsl2581.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c_tsl2581.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	i2c_tsl2581.Init.OwnAddress1 = 0x00;
	i2c_tsl2581.Init.OwnAddress2 = 0x00;

	i2c_tsl2581.Instance = I2C1;
	i2c_tsl2581.Mode = HAL_I2C_MODE_MASTER;

	tsl2581.hi2c = &i2c_tsl2581;
	(HAL_I2C_Init(tsl2581.hi2c));

	tsl2581_start(&tsl2581);

	trace_printf("LS: %d\n", error);
}



void TSL_Task()
{
	for(;;)
	{
		vTaskDelay(250 / portTICK_RATE_MS);
	taskENTER_CRITICAL();
		tsl2581_readADC(&tsl2581, &data_TSL.ch0, &data_TSL.ch1);
		data_TSL.time = HAL_GetTick();
		tsl2581_calcLux(&tsl2581, &data_TSL.lux, &data_TSL.ch0, &data_TSL.ch1);
	taskEXIT_CRITICAL();
		vTaskDelay(250 / portTICK_RATE_MS);
		writeDataTSL();
		//trace_printf("lux: %d\nch0: %d\nch1: %d\n===========\n", data_TSL.lux, data_TSL.ch0, data_TSL.ch1);
	}
}
