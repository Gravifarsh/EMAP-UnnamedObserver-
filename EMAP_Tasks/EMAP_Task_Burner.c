/*
 * EMAP_Task_Burner.c
 *
 *  Created on: 29 июн. 2019 г.
 *      Author: developer
 */

#include "FreeRTOS.h"
#include "task.h"

#include "EMAP_Task_Burner.h"

void BURNER_Init()
{
	__GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef gpioa;

	gpioa.Mode = GPIO_MODE_OUTPUT_PP;
	gpioa.Pin = GPIO_PIN_8;
	gpioa.Pull = GPIO_NOPULL;
	gpioa.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &gpioa);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

void BURNER_Task()
{
	uint32_t startTick, tick;
	uint8_t start = 0;

	for(;;)
	{
		taskENTER_CRITICAL();
		uint16_t brightness = data_TSL.ch0;
		tick = HAL_GetTick();
		taskEXIT_CRITICAL();
		switch (system_state.GlobalState)
			{
			case EMAP_STATE_READY:
				if (brightness < TSL_DARK)
				{
					if (!start)
					{
						startTick = tick;
						start = !start;
					}
					else
					{
						if (tick - startTick > PAYLOAD_TIME)
						{
							system_state.GlobalState = EMAP_STATE_PAYLOAD;
							trace_printf("State: %d\n", system_state.GlobalState);
							startTick = 0;
							tick = 0;
							start = !start;
						}
					}
				}
				break;

			case EMAP_STATE_PAYLOAD:
				if (brightness > TSL_BRIGHT)
				{
					if (!start)
					{
						startTick = tick;
						start = !start;
					}
					else
					{
						if (tick - startTick > FALLING_TIME)
						{
							system_state.GlobalState = EMAP_STATE_FALLING;
							trace_printf("State: %d\n", system_state.GlobalState);
							startTick = 0;
							tick = 0;
							start = !start;
						}
					}
				}
				break;

			case EMAP_STATE_FALLING:
				trace_printf("height1: %d\nheight2: %d\n", data_BMP280_1.height, data_BMP280_2.height);
				if((data_BMP280_1.height + data_BMP280_2.height) / 2 < 300)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
					vTaskDelay(3000);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
					vTaskDelete(NULL);
				}
				break;
			}
		vTaskDelay(1500);
	}
}
