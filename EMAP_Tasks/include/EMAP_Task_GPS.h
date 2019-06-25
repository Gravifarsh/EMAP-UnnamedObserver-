/*
 * EMAP_Task_GPS.h
 *
 *  Created on: 25 июн. 2019 г.
 *      Author: developer
 */

#ifndef INCLUDE_EMAP_TASK_GPS_H_
#define INCLUDE_EMAP_TASK_GPS_H_

#include <stm32f4xx_hal.h>

#include "EMAPConfig.h"

#define GPS_USART 				(USART2)
#define GPS_DMA_BUFFER_SIZE 	(400)
#define GPS_MSG_BUFFER_SIZE 	(200)
#define GPS_DMA_USART_STREAM 	(DMA1_Stream5)

#define GPS_BUFFER_SIZE			(100)

extern uint8_t dma_usartBuffer[100];

void GPS_Init();
void GPS_Task();

#endif /* INCLUDE_EMAP_TASK_GPS_H_ */
