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
		switch (system_state.GlobalState)
			{
			case EMAP_STATE_READY:
				if (data_TSL.ch0 < TSL_DARK)
				{
				taskENTER_CRITICAL();
					if (!start)
					{
						startTick = HAL_GetTick();
						start = !start;
					}
					else
					{
						tick = HAL_GetTick();
						if (tick - startTick > PAYLOAD_TIME)
						{
							system_state.GlobalState = EMAP_STATE_PAYLOAD;
							trace_printf("State: %d\n", system_state.GlobalState);
							startTick = 0;
							tick = 0;
							start = !start;
						}
					}
				taskEXIT_CRITICAL();
				}
				break;

			case EMAP_STATE_PAYLOAD:
				if (data_TSL.ch0 > TSL_BRIGHT)
				{
				taskENTER_CRITICAL();
					if (!start)
					{
						startTick = HAL_GetTick();
						start = !start;
					}
					else
					{
						tick = HAL_GetTick();
						if (tick - startTick > FALLING_TIME)
						{
							system_state.GlobalState = EMAP_STATE_FALLING;
							trace_printf("State: %d\n", system_state.GlobalState);
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
							HAL_Delay(1500);
							//vTaskDelay(2000);
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
							startTick = 0;
							tick = 0;
							start = !start;
						}
					}
				taskEXIT_CRITICAL();
				}
				break;
			}
		vTaskDelay(1500);
	}
}
