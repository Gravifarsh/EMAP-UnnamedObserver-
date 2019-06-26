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

HAL_StatusTypeDef tsl2561_readReg(tsl2561_t * htsl, uint8_t regAddr, uint8_t * buffer, uint8_t size)
{
	return HAL_I2C_Mem_Read(htsl->hi2c, htsl->addr/*TSL2561_ADDR_FLOAT*/, regAddr, I2C_MEMADD_SIZE_8BIT, buffer, size, 0xFF);
}

HAL_StatusTypeDef tsl2561_writeReg(tsl2561_t * htsl, uint8_t regAddr, uint8_t buffer, uint8_t size)
{
	HAL_StatusTypeDef error = 0;
	uint8_t regData = 0x00;
	PROCESS_ERROR(tsl2561_readReg(htsl, regAddr, &regData, 1));

	uint8_t regData_new = (regData | buffer);
	return HAL_I2C_Mem_Write(htsl->hi2c, htsl->addr/*TSL2561_ADDR_FLOAT*/, regAddr, I2C_MEMADD_SIZE_8BIT, &regData_new, size, 0xFF);

end:
	return error;
}


HAL_StatusTypeDef tsl2561_readADC(tsl2561_t * htsl, uint16_t * ch0, uint16_t * ch1)
{
	HAL_StatusTypeDef error = 0;
	uint8_t buf[2];

	uint8_t dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_CHAN0_LOW;
	PROCESS_ERROR( tsl2561_readReg(htsl, dummy, &buf[1], 1) );
	dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_CHAN0_HIGH;
	PROCESS_ERROR( tsl2561_readReg(htsl, dummy, &buf[0], 1) );
	*ch0 = buf[0] * 256 + buf[1];

	dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_CHAN1_LOW;
	PROCESS_ERROR( tsl2561_readReg(htsl, dummy, &buf[1], 1) );
	dummy = TSL2561_COMMAND_BIT | TSL2561_BLOCK_BIT | TSL2561_REGISTER_CHAN1_HIGH;
	PROCESS_ERROR( tsl2561_readReg(htsl, dummy, &buf[0], 1) );
	*ch1 = buf[0] * 256 + buf[1];

end:
	return error;
}

void tsl2561_calcLux(tsl2561_t * htsl, unsigned int * lux,uint16_t * ch0, uint16_t * ch1)
{
	unsigned long scale;
	unsigned long channel0;
	unsigned long channel1;

	switch (htsl->intg)		//Scalig
	{
	case TSL2561_INT_13MS:
		scale = CHSCALE_TINT0;
		break;
	case TSL2561_INT_100MS:
		scale = CHSCALE_TINT1;
		break;
	default:
		scale = (1 << CH_SCALE);
		break;
	}

	if (!htsl->gain)
		scale = scale << 4;

	channel0 = ((*ch0) * scale) >> CH_SCALE;
	channel1 = ((*ch1) * scale) >> CH_SCALE;

	unsigned long ratio1 = 0;
	if (channel0 != 0) ratio1 = (channel1 << (RATIO_SCALE+1)) / channel0;
	// round the ratio value
	unsigned long ratio = (ratio1 + 1) >> 1;
	unsigned int b = 0, m = 0;

	switch (htsl->type)
	{
	case TSL2561_TYPE_T:// T, FN and CL package
		if (ratio <= K1T)
		{b=B1T; m=M1T;}
		else if (ratio <= K2T)
		{b=B2T; m=M2T;}
		else if (ratio <= K3T)
		{b=B3T; m=M3T;}
		else if (ratio <= K4T)
		{b=B4T; m=M4T;}
		else if (ratio <= K5T)
		{b=B5T; m=M5T;}
		else if (ratio <= K6T)
		{b=B6T; m=M6T;}
		else if (ratio <= K7T)
		{b=B7T; m=M7T;}
		else if (ratio > K8T)
		{b=B8T; m=M8T;}
		break;
	case TSL2561_TYPE_CS:// CS package
		if (ratio <= K1C)
		{b=B1C; m=M1C;}
		else if (ratio <= K2C)
		{b=B2C; m=M2C;}
		else if (ratio <= K3C)
		{b=B3C; m=M3C;}
		else if (ratio <= K4C)
		{b=B4C; m=M4C;}
		else if (ratio <= K5C)
		{b=B5C; m=M5C;}
		else if (ratio <= K6C)
		{b=B6C; m=M6C;}
		else if (ratio <= K7C)
		{b=B7C; m=M7C;}
		else if (ratio > K8C)
		{b=B8C; m=M8C;}
		break;
	}

	unsigned long temp;
	temp = ((channel0 * b) - (channel1 * m));
	// do not allow negative lux value
	// round lsb (2^(LUX_SCALE−1))
	temp += (1 << (LUX_SCALE - 1));
	// strip off fractional portion
	*lux = temp >> LUX_SCALE;
}
