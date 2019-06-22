/*
 * lidar.h
 *
 *  Created on: 11 июн. 2019 г.
 *      Author: developer
 */

#ifndef LIDAR_LIDAR_H_
#define LIDAR_LIDAR_H_

#include "inttypes.h"
#include "stm32f4xx_hal.h"

#define LIDAR_RXBUF_SIZE (256)
#define LIDAR_TXBUF_SIZE (32)

typedef struct {
	UART_HandleTypeDef *huart;

	uint32_t repint_freq;

	char txbuffer[LIDAR_TXBUF_SIZE];
	char rxbuffer[LIDAR_RXBUF_SIZE];
} lidar_t;

HAL_StatusTypeDef lidar_tdcInit(lidar_t*);
HAL_StatusTypeDef lidar_setMeasFreq(lidar_t*, uint32_t);
HAL_StatusTypeDef lidar_setRepIntFreq(lidar_t*, uint32_t);

HAL_StatusTypeDef lidar_meas(lidar_t*, uint32_t*);

HAL_StatusTypeDef lidar_start(lidar_t*);
HAL_StatusTypeDef lidar_stop(lidar_t*);
HAL_StatusTypeDef lidar_getRes(lidar_t*, uint32_t*, uint32_t*, uint32_t*);

#endif /* LIDAR_LIDAR_H_ */
