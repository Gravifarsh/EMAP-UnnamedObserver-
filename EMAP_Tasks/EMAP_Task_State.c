/*
 * EMAP_Task_State.c
 *
 *  Created on: 2 июл. 2019 г.
 *      Author: developer
 */

#include "FreeRTOS.h"
#include "task.h"

void STATE_Task()
{
	for(;;)
	{
		vTaskDelay(1000 / portTICK_RATE_MS);
		writeSysState();
	}
}
