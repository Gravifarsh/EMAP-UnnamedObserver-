/*
 * EMAP_Task_SD.c
 *
 *  Created on: 19 апр. 2019 г.
 *      Author: developer
 */

#include "stdint.h"

#include "diag/Trace.h"
#include "FreeRTOS.h"
#include "task.h"

#include "EMAPConfig.h"
#include "EMAP_Task_SD.h"
#include "dump.h"

#include "EMAP_Task_RF.h"

#define NEED_ACK	false

static dump_channel_state_t stream_file;

void SD_Init()
{
	//uint8_t nRF24L01_initError = nRF24L01_init(&spi_nRF24L01);
	//system_state.nRF = nRF24L01_initError;
	HAL_Delay(100);

	stream_file.res = 1;
	stream_file.file_opened = false;
	dump_init(&stream_file);
taskENTER_CRITICAL();
	system_state.SD = (uint8_t)stream_file.res;
taskEXIT_CRITICAL();
	HAL_Delay(200);
}

void writeSysStateZero()
{
	uint8_t buffer[sizeof(system_state_zero_t) + 2];

	buffer[0] = 0xF9;
	memcpy(&buffer[2], &system_state_zero, sizeof(system_state_zero_t));
	buffer[sizeof(system_state_zero_t) + 1] = 0xF9;

	//trace_puts("Sending zero");
	//nRF24L01_send(&spi_nRF24L01, buffer, PACKET_LEN_SYS_STATE_ZERO, NEED_ACK);
	dump(&stream_file, buffer, sizeof(system_state_zero_t) + 2);
	send(buffer, sizeof(system_state_zero_t) + 2);
}

void writeSysState()
{
	uint8_t buffer[sizeof(system_state_t) + 2];

taskENTER_CRITICAL();
	system_state.SD = stream_file.res;

	buffer[0] = 0xFF;
	memcpy(buffer + 1, &system_state, sizeof(system_state_t));
	buffer[sizeof(system_state_t) + 1] = 0xFF;
taskEXIT_CRITICAL();

	dump(&stream_file, buffer, sizeof(system_state_t) + 2);
	send(buffer, sizeof(system_state_t) + 2);
}

void writeDataMPU(I2C_HandleTypeDef * hi2c)
{
	uint8_t buffer[sizeof(data_MPU9255_t) + 2];

taskENTER_CRITICAL();
	if(hi2c->Instance == I2C1)
	{
		buffer[0] = 0xFA;
		memcpy(buffer + 1, &data_MPU9255_1, sizeof(data_MPU9255_t));
		buffer[sizeof(data_MPU9255_1) + 1] = 0xFA;
	}
	else
	{
		buffer[0] = 0xFB;
		memcpy(buffer + 1, &data_MPU9255_2, sizeof(data_MPU9255_t));
		buffer[sizeof(data_MPU9255_2) + 1] = 0xFB;
	}
taskEXIT_CRITICAL();

	//trace_puts("Sending IMU");
	dump(&stream_file, buffer, sizeof(data_MPU9255_t) + 2);
	send(buffer, sizeof(data_MPU9255_t) + 2);
}

void writeDataIsc()
{
	uint8_t buffer[sizeof(data_MPU9255_t) + 2];

taskENTER_CRITICAL();
	buffer[0] = 0xFC;
	memcpy(buffer + 1, &data_MPU9255_isc, sizeof(data_MPU9255_t));
	buffer[sizeof(data_MPU9255_t) + 1] = 0xFC;
taskEXIT_CRITICAL();

	//trace_puts("Sending ISC");
	//nRF24L01_send(&spi_nRF24L01, buffer, PACKET_LEN_DATA_MPU, NEED_ACK);
	dump(&stream_file, buffer, sizeof(data_MPU9255_t) + 2);
	send(buffer, sizeof(data_MPU9255_t) + 2);
}

void writeDataBMP()
{
	uint8_t buffer[sizeof(data_BMP280_t) + 2];

taskENTER_CRITICAL();
	buffer[0] = 0xFD;
	memcpy(buffer + 1, &data_BMP280_1, sizeof(data_BMP280_t));
	buffer[sizeof(data_BMP280_t) + 1] = 0xFD;
taskEXIT_CRITICAL();

	//trace_puts("Sending BMP1");
	dump(&stream_file, buffer, sizeof(data_BMP280_t) + 2);
	send(buffer, sizeof(data_BMP280_t) + 2);

taskENTER_CRITICAL();
	buffer[0] = 0xFE;
	memcpy(buffer + 1, &data_BMP280_2, sizeof(data_BMP280_t));
	buffer[sizeof(data_BMP280_t) + 1] = 0xFE;
taskEXIT_CRITICAL();

	//trace_puts("Sending BMP1");
	dump(&stream_file, buffer, sizeof(data_BMP280_t) + 2);
	send(buffer, sizeof(data_BMP280_t) + 2);
}

void SD_Task()
{
	bool isZeroWrote = false;


	if(stream_file.file_opened == false)
	{
		trace_printf("SD_Task Shut Down!\n");
		vTaskDelete(NULL);
	}

	trace_printf("SD State: %d\n", system_state.SD);

	for(;;)
	{
		//trace_puts("SD TASK");
		vTaskDelay(100/portTICK_RATE_MS);

		if(isZeroWrote)
		{
			writeSysState();
			writeDataMPU(&i2c_IMU_1);
			writeDataMPU(&i2c_IMU_2);
			writeDataIsc();
			writeDataBMP();
		}
		else
		{
			if(system_state_zero.pressure)
			{
				writeSysStateZero();
				isZeroWrote = true;
			}
		}
	}
}
