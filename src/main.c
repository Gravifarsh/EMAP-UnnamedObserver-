/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <DATA_Helper.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stm32f4xx_hal.h>

#include "diag/Trace.h"
#include "FreeRTOS.h"
#include "task.h"

#include "EMAPConfig.h"
#include "EMAP_Task_IMU.h"
#include "EMAP_Task_GPS.h"
#include "EMAP_Task_TSL.h"
#include "EMAP_Task_LIDAR.h"


//	параметры GPS_task
#define GPS_TASK_STACK_SIZE	(20*configMINIMAL_STACK_SIZE)
static StackType_t	_GPSTaskStack[GPS_TASK_STACK_SIZE];
static StaticTask_t	_GPSTaskObj;

//	параметры IMU_task
#define IMU_TASK_STACK_SIZE (20*configMINIMAL_STACK_SIZE)
static StackType_t	_IMUTaskStack[IMU_TASK_STACK_SIZE];
static StaticTask_t	_IMUTaskObj;

//	параметры TSL_task
#define TSL_TASK_STACK_SIZE (10*configMINIMAL_STACK_SIZE)
static StackType_t	_TSLTaskStack[TSL_TASK_STACK_SIZE];
static StaticTask_t	_TSLTaskObj;

//	параметры LIDAR_task
#define LIDAR_TASK_STACK_SIZE (10*configMINIMAL_STACK_SIZE)
static StackType_t	_LIDARTaskStack[LIDAR_TASK_STACK_SIZE];
static StaticTask_t	_LIDARTaskObj;


I2C_HandleTypeDef 	i2c_IMU_1;
I2C_HandleTypeDef 	i2c_IMU_2;
SPI_HandleTypeDef	spi_nRF24L01;
I2C_HandleTypeDef 	i2c_IMU_2;
rscs_bmp280_descriptor_t * IMU_bmp280_1;
rscs_bmp280_descriptor_t * IMU_bmp280_2;

data_raw_BMP280_t 	data_raw_BMP280_1;
data_raw_BMP280_t 	data_raw_BMP280_2;
data_MPU9255_t 		data_MPU9255_isc;
data_MPU9255_t 		data_MPU9255_1;
data_MPU9255_t 		data_MPU9255_2;
data_BMP280_t 		data_BMP280_1;
data_BMP280_t 		data_BMP280_2;
data_TSL_t			data_TSL;
data_GPS_t			data_GPS;
system_state_t 		system_state;
system_state_zero_t system_state_zero;

data_MPU9255_t 	data_prev_MPU9255_1;
data_MPU9255_t 	data_prev_MPU9255_isc;
data_MPU9255_t 	data_prev_MPU9255_2;
system_state_t 	system_prev_state;

// ----------------------------------------------------------------------------
//
// Standalone STM32F4 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
//#pragma GCC optimize("Ofast, unroll-all-loops")
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


int
main(int argc, char* argv[])
{
	/* FILL WITH ZEROS */
	FILL_STRUCT_WITH_ZERO(data_raw_BMP280_1)
	FILL_STRUCT_WITH_ZERO(data_BMP280_1)

	FILL_STRUCT_WITH_ZERO(data_raw_BMP280_2)
	FILL_STRUCT_WITH_ZERO(data_BMP280_2)

	FILL_STRUCT_WITH_ZERO(data_MPU9255_1)
	FILL_STRUCT_WITH_ZERO(data_MPU9255_2)
	FILL_STRUCT_WITH_ZERO(data_MPU9255_isc)

	FILL_STRUCT_WITH_ZERO(data_GPS)

	FILL_STRUCT_WITH_ZERO(system_state)
	FILL_STRUCT_WITH_ZERO(system_state_zero)

	FILL_STRUCT_WITH_ZERO(data_prev_MPU9255_1)
	FILL_STRUCT_WITH_ZERO(data_prev_MPU9255_2)
	FILL_STRUCT_WITH_ZERO(data_prev_MPU9255_isc)

	FILL_STRUCT_WITH_ZERO(system_prev_state)

	FILL_STRUCT_WITH_ZERO(i2c_IMU_1)
	FILL_STRUCT_WITH_ZERO(i2c_IMU_2)
	FILL_STRUCT_WITH_ZERO(i2c_tsl2581)

	FILL_STRUCT_WITH_ZERO(spi_nRF24L01)

	FILL_STRUCT_WITH_ZERO(uart_lidar)

	FILL_STRUCT_WITH_ZERO(IMU_bmp280_1)
	FILL_STRUCT_WITH_ZERO(IMU_bmp280_2)

	FILL_STRUCT_WITH_ZERO(nRF24)

	FILL_STRUCT_WITH_ZERO(tsl2581)

	FILL_STRUCT_WITH_ZERO(lidar)

	/* FIRERISER */
	GPIO_InitTypeDef gpio;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_PULLDOWN;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOA, &gpio);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

	/* CREATING TASKS */
	xTaskCreateStatic(IMU_Task, 	"IMU",		IMU_TASK_STACK_SIZE, 		NULL, 4, _IMUTaskStack,		&_IMUTaskObj);
	xTaskCreateStatic(GPS_Task, 	"GPS",		GPS_TASK_STACK_SIZE, 		NULL, 1, _GPSTaskStack,		&_GPSTaskObj);
	xTaskCreateStatic(TSL_Task, 	"TSL", 		TSL_TASK_STACK_SIZE, 		NULL, 2, _TSLTaskStack,		&_TSLTaskObj);
	xTaskCreateStatic(LIDAR_Task,	"LIDAR",	LIDAR_TASK_STACK_SIZE,		NULL, 3, _LIDARTaskStack, 	&_LIDARTaskObj);

	/* CALLING INITS */
	DATA_Init();

	IMU_Init();

	GPS_Init();

	TSL_Init();

	LIDAR_Init();

	/* STARTING */
	vTaskStartScheduler();
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
