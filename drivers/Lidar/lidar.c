/*
 * lidar.c
 *
 *  Created on: 11 июн. 2019 г.
 *      Author: developer
 */
#include "lidar.h"

#include <string.h>

#define COM_TDCINIT ("_tdcinit;")
#define COM_MEAS ("_meas;")
#define COM_START ("_start;")
#define COM_STOP ("_stop;")

HAL_StatusTypeDef lidar_tdcInit(lidar_t* dev) {
	return HAL_OK;
}

HAL_StatusTypeDef lidar_setMeasFreq(lidar_t* dev, uint64_t freq) {
	return HAL_OK;
}

HAL_StatusTypeDef lidar_meas(lidar_t* dev, uint32_t* res) {
	return HAL_OK;
}

HAL_StatusTypeDef lidar_start(lidar_t* dev) {
	return HAL_OK;
}

HAL_StatusTypeDef lidar_stop(lidar_t* dev) {
	return HAL_OK;
}
