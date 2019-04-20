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

static dump_channel_state_t stream_file;

void SD_Init()
{
	stream_file.res = 1;
	stream_file.file_opened = false;
	dump_init(&stream_file);
	system_state.SD = (uint8_t)stream_file.res;
	HAL_Delay(200);
}

//TODO: Таск записи на Sd карту: zero state(один раз), (в цикле)system_state, data_MPU9255_1, data_MPU9255_2, data_MPU9255_isc, data_BMP280_1, data_BMP280_2
