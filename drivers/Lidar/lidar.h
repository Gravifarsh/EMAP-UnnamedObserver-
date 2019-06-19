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

typedef struct {
	UART_HandleTypeDef *huart;
} lidar_t;

HAL_StatusTypeDef lidar_tdcInit(lidar_t*);
HAL_StatusTypeDef lidar_setMeasFreq(lidar_t*, uint64_t);
HAL_StatusTypeDef lidar_meas(lidar_t*, uint32_t*);
HAL_StatusTypeDef lidar_start(lidar_t*);
HAL_StatusTypeDef lidar_stop(lidar_t*);

#endif /* LIDAR_LIDAR_H_ */
