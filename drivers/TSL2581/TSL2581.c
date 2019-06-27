/*
 * TSL2581.c
 *
 *  Created on: 1 июн. 2019 г.
 *      Author: developer
 */

#include "../TSL2581/TSL2581.h"
#include "diag/Trace.h"


I2C_HandleTypeDef	i2c_tsl2581;
tsl2581_t			tsl2581;

#define CMD_PREAMBLE 0x80 // 0b10010000

static HAL_StatusTypeDef tsl2581_readReg8(tsl2581_t * htsl, uint8_t regAddr, uint8_t * value)
{
	uint8_t command = CMD_PREAMBLE | regAddr;
	return HAL_I2C_Mem_Read(htsl->hi2c, htsl->addr, command, I2C_MEMADD_SIZE_8BIT, value, 1, 0xFF);
}

static HAL_StatusTypeDef tsl2581_writeReg8(tsl2581_t * htsl, uint8_t regAddr, uint8_t value)
{
	uint8_t command = CMD_PREAMBLE | regAddr;
	HAL_StatusTypeDef error = HAL_I2C_Mem_Write(htsl->hi2c, htsl->addr, command, I2C_MEMADD_SIZE_8BIT, &value, 1, 0xFF);
	uint8_t dummy = 0xFF;
	error = tsl2581_readReg8(htsl, regAddr, &dummy);
	return error;
}


static HAL_StatusTypeDef tsl2581_readReg16(tsl2581_t * htsl, uint8_t regAddr, uint16_t * value)
{
	uint8_t command = CMD_PREAMBLE | regAddr;
	return HAL_I2C_Mem_Read(htsl->hi2c, htsl->addr, command, I2C_MEMADD_SIZE_8BIT, (uint8_t*)value, 2, 0xFF);
	uint8_t * ptr = (uint8_t*)value;
	*value = ptr[0] * 256 + ptr[1];
}


HAL_StatusTypeDef tsl2581_start(tsl2581_t * htsl)
{
	HAL_StatusTypeDef error = HAL_OK;
	uint8_t dummy = 0xff;

	error = tsl2581_readReg8(htsl, TSL2581_REGISTER_CONTROL, &dummy);

	error = tsl2581_writeReg8(htsl, TSL2581_REGISTER_CONTROL, CONTROL_REG_VALUE_POWER_UP);
	if (error != HAL_OK)
		return error;

	error = tsl2581_readReg8(htsl, TSL2581_REGISTER_CONTROL, &dummy);
	error = tsl2581_readReg8(htsl, TSL2581_REGISTER_ANALOG, &dummy);

	error = tsl2581_writeReg8(htsl, TSL2581_REGISTER_ANALOG, htsl->gain);
	if (error != HAL_OK)
			return error;

	error = tsl2581_readReg8(htsl, TSL2581_REGISTER_ANALOG, &dummy);
	error = tsl2581_readReg8(htsl, TSL2581_REGISTER_TIMING, &dummy);

	error = tsl2581_writeReg8(htsl, TSL2581_REGISTER_TIMING, htsl->intg);
	if (error != HAL_OK)
			return error;

	error = tsl2581_readReg8(htsl, TSL2581_REGISTER_TIMING, &dummy);
	error = tsl2581_readReg8(htsl, TSL2581_REGISTER_CONTROL, &dummy);

	error = tsl2581_writeReg8(htsl, TSL2581_REGISTER_CONTROL, CONTROL_REG_VALUE_ADC_UP | CONTROL_REG_VALUE_POWER_UP);
	if (error != HAL_OK)
			return error;

	error = tsl2581_readReg8(htsl, TSL2581_REGISTER_CONTROL, &dummy);

	return error;
}

HAL_StatusTypeDef tsl2581_readADC(tsl2581_t * htsl, uint16_t * ch0, uint16_t * ch1)
{
	HAL_StatusTypeDef error = HAL_OK;

	error = tsl2581_readReg16(htsl, TSL2581_REGISTER_CHAN0_LOW, ch0);
	if (error != HAL_OK)
		return error;

	error = tsl2581_readReg16(htsl, TSL2581_REGISTER_CHAN1_LOW, ch1);
	if (error != HAL_OK)
		return error;

	return error;
}


void tsl2581_calcLux(tsl2581_t * htsl, unsigned int * lux, uint16_t * ch0, uint16_t * ch1)
{
	unsigned int channel0;
	unsigned int channel1;
	unsigned int ch0Scale;
	unsigned int ch1Scale;

	if (htsl->intg == TSL2581_INT_399MS)
		ch0Scale = (1 << CH_SCALE);
	else
		ch0Scale = ((TSL2581_INT_399MS << CH_SCALE) / htsl->intg);

	switch (htsl->gain) {
		case 0:
			// 1x gain: no scale, nominal setting
			ch1Scale = ch0Scale; //
			break;
		case 1:
			// 8x gain: scale/multiply value by 1/8
			ch0Scale = ch0Scale>> 3;
			ch1Scale = ch0Scale;
			break;
		case 2: // 16x gain: scale/multiply value by 1/16
			ch0Scale = ch0Scale >> 4;
			ch1Scale = ch0Scale;
			break;
		case 3: // 128x gain: CH0/CH1 gain correction factors applied
			ch1Scale = ch0Scale / TSL2581_CH1GAIN128X;
			ch0Scale = ch0Scale / TSL2581_CH0GAIN128X;
		break;
	}

	channel0 = ((*ch0) * ch0Scale) >> CH_SCALE;
	channel1 = ((*ch1) * ch1Scale) >> CH_SCALE;

	unsigned long ratio1 = 0;
	unsigned int b = 0, m = 0;
	// Find the ratio of the channel values (Channel1/Channel0), protect against divide by zero
	if (channel0) ratio1 = (((channel1 << (RATIO_SCALE + 1)) / channel0) + 1) >> 1;
	if ((ratio1 >= 0) && (ratio1 <= K1C)) {
		b = B1C;
		m = M1C;
	} else if (ratio1 <= K2C) {
		b = B2C;
		m = M2C;
	} else if (ratio1 <= K3C) {
		b = B3C;
		m = M3C;
	} else if (ratio1 <= K4C) {
		b = B4C;
		m = M4C;
	} else if (ratio1 >  K5C) {
		b = B5C;
		m = M5C;
	}

	unsigned long temp;
	temp = ((channel0 * b) - (channel1 * m));
	// round lsb (2^(LUX_SCALE−1))
	temp += (1 << (LUX_SCALE - 1));
	// strip off fractional portion
	*lux = temp >> LUX_SCALE;
}
