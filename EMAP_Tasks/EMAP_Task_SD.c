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
#include "nRF24L01/nRF24L01.h"
#include "dump.h"

#define NEED_ACK	false

static dump_channel_state_t stream_file;

void SD_Init()
{
	uint8_t nRF24L01_initError = nRF24L01_init(&spi_nRF24L01);
	system_state.nRF = nRF24L01_initError;
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
	uint8_t buffer[PACKET_LEN_SYS_STATE_ZERO];

	buffer[0] = 0xFF;
	buffer[1] = 0xF0;
	memcpy(&buffer[2], &system_state_zero, sizeof(system_state_zero));
	buffer[PACKET_LEN_SYS_STATE_ZERO - 1] = 0xFF;

	trace_puts("Sending zero\n");
	nRF24L01_send(&spi_nRF24L01, buffer, PACKET_LEN_SYS_STATE_ZERO, NEED_ACK);
//	dump(&stream_file, buffer, PACKET_LEN_SYS_STATE_ZERO);
}

void writeSysState()
{
	uint8_t buffer[PACKET_LEN_SYS_STATE];

taskENTER_CRITICAL();
	system_state.SD = stream_file.res;

	buffer[0] = 0xFF;
	buffer[1] = 0xF1;
	memcpy(&buffer[2], &system_state, sizeof(system_state));
	buffer[PACKET_LEN_SYS_STATE - 1] = 0xFF;
taskEXIT_CRITICAL();

//	dump(&stream_file, buffer, PACKET_LEN_SYS_STATE);
}

void writeDataMPU(I2C_HandleTypeDef * hi2c)
{
	uint8_t buffer[PACKET_LEN_DATA_MPU];

taskENTER_CRITICAL();
	float time = (float)HAL_GetTick() / 1000;
	buffer[0] = 0xFF;
	memcpy(&buffer[2], &time, sizeof(float));
	if(hi2c->Instance == I2C1)
	{
		buffer[1] = 0xF2;
		memcpy(&buffer[6], &data_MPU9255_1, sizeof(data_MPU9255_1));
	}
	else
	{
		buffer[1] = 0xF3;
		memcpy(&buffer[6], &data_MPU9255_2, sizeof(data_MPU9255_2));
	}
	buffer[PACKET_LEN_DATA_MPU - 1] = 0xFF;
taskEXIT_CRITICAL();

	trace_puts("Sending IMU\n");
	nRF24L01_send(&spi_nRF24L01, buffer, PACKET_LEN_DATA_MPU, NEED_ACK);
//	dump(&stream_file, buffer, PACKET_LEN_DATA_MPU);
}

void writeDataIsc()
{
	uint8_t buffer[PACKET_LEN_DATA_MPU];

taskENTER_CRITICAL();
	float time = (float)HAL_GetTick() / 1000;
	buffer[0] = 0xFF;
	buffer[1] = 0xF4;
	memcpy(&buffer[2], &time, sizeof(float));
	memcpy(&buffer[6], &data_MPU9255_isc, sizeof(data_MPU9255_isc));
	buffer[PACKET_LEN_DATA_MPU - 1] = 0xFF;
taskEXIT_CRITICAL();

	trace_puts("Sending ISC\n");
	nRF24L01_send(&spi_nRF24L01, buffer, PACKET_LEN_DATA_MPU, NEED_ACK);
//	dump(&stream_file, buffer, PACKET_LEN_DATA_MPU);
}

void writeDataBMP()
{
	uint8_t buffer[PACKET_LEN_DATA_BMP];

taskENTER_CRITICAL();
	float time = (float)HAL_GetTick() / 1000;
	buffer[0] = 0xFF;
	buffer[1] = 0xF5;
	memcpy(&buffer[2], &time, sizeof(float));
	memcpy(&buffer[6], &data_BMP280_1, sizeof(data_BMP280_1));
	buffer[PACKET_LEN_DATA_BMP - 1] = 0xFF;
taskEXIT_CRITICAL();

	trace_puts("Sending BMP1\n");
	nRF24L01_send(&spi_nRF24L01, buffer, PACKET_LEN_DATA_BMP, NEED_ACK);
//	dump(&stream_file, buffer, PACKET_LEN_DATA_BMP);

taskENTER_CRITICAL();
	buffer[1] = 0xF6;
	memcpy(&buffer[6], &data_BMP280_1, sizeof(data_BMP280_1));
taskEXIT_CRITICAL();

	trace_puts("Sending BMP1\n");
	nRF24L01_send(&spi_nRF24L01, buffer, PACKET_LEN_DATA_BMP, NEED_ACK);
//	dump(&stream_file, buffer, PACKET_LEN_DATA_BMP);
}

void SD_Task()
{
	_Bool isZeroWrote = false;

	/*
	if(stream_file.file_opened == false)
	{
		trace_printf("SD_Task Shut Down!\n");
		vTaskDelete(NULL);
	}
	*/

	//trace_printf("SD State: %d\n", system_state.SD);
	trace_printf("nRF State: %d\n", system_state.nRF);

	for(;;)
	{
		vTaskDelay(100/portTICK_RATE_MS);
//		trace_printf("SD here\n");

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
			//if(system_state_zero.pressure)
			{
				writeSysStateZero();
				isZeroWrote = true;
			}
		}
	}
}
