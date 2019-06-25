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

HAL_StatusTypeDef tsl2561_readReg(I2C_HandleTypeDef * hi2c, uint8_t regAddr, uint8_t * buffer, uint8_t size)
{
	return HAL_I2C_Mem_Read(hi2c, TSL2561_ADDR_FLOAT, regAddr, I2C_MEMADD_SIZE_8BIT, buffer, size, 0xFF);
}

HAL_StatusTypeDef tsl2561_writeReg(I2C_HandleTypeDef * hi2c, uint8_t regAddr, uint8_t buffer, uint8_t size)
{
	int error = 0;
	uint8_t regData = 0x00;
	PROCESS_ERROR(tsl2561_readReg(hi2c, regAddr, &regData, 1));

	uint8_t regData_new = (regData | buffer);
	return HAL_I2C_Mem_Write(hi2c, TSL2561_ADDR_FLOAT, regAddr, I2C_MEMADD_SIZE_8BIT, &regData_new, size, 0xFF);

end:
	return error;
}
