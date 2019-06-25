/*
 * EMAP_Task_LIDAR.c
 *
 *  Created on: 25 июн. 2019 г.
 *      Author: developer
 */


#include "EMAP_Task_LIDAR.h"

UART_HandleTypeDef	uart_lidar;
lidar_t				lidar;

void LIDAR_Task() {
	uint32_t dummy;
	for(;;) {
		trace_printf("%d\n", lidar_meas(&lidar, &dummy));

		trace_printf("%s\n", lidar.rxbuffer);
	}
}

void LIDAR_Init() {
	uart_lidar.Instance = UART4;
	uart_lidar.Init.BaudRate = 38400;
	uart_lidar.Init.Mode = UART_MODE_TX_RX;
	uart_lidar.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart_lidar.Init.OverSampling = UART_OVERSAMPLING_8;
	uart_lidar.Init.Parity = UART_PARITY_NONE;
	uart_lidar.Init.StopBits = UART_STOPBITS_1;
	uart_lidar.Init.WordLength = UART_WORDLENGTH_8B;

	HAL_UART_Init(&uart_lidar);

	lidar.huart = &uart_lidar;

	trace_printf("LIDAR: %d\n", lidar_tdcInit(&lidar));
}
