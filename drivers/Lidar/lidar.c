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
#define COM_MEASFREQ ("_measfreq=%d;")
#define COM_REPINTFREQ ("_repint=%d;")

#define RES_ACK ("ACK")

#define RX_TIMEOUT (100)
#define TX_TIMEOUT (100)

HAL_StatusTypeDef lidar_tdcInit(lidar_t* dev) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);

	HAL_UART_Transmit(dev->huart, COM_TDCINIT, strlen(COM_TDCINIT), TX_TIMEOUT);

	HAL_UART_Receive(dev->huart, dev->rxbuffer, LIDAR_RXBUF_SIZE - 1, RX_TIMEOUT);

	if(strstr(dev->rxbuffer, RES_ACK)) return HAL_OK;
	return HAL_ERROR;
}

HAL_StatusTypeDef lidar_setMeasFreq(lidar_t* dev, uint64_t freq) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);

	sprintf(dev->txbuffer, COM_MEASFREQ, freq);

	HAL_UART_Transmit(dev->huart, dev->txbuffer, strlen(dev->txbuffer), TX_TIMEOUT);

	HAL_UART_Receive(dev->huart, dev->rxbuffer, LIDAR_RXBUF_SIZE - 1, RX_TIMEOUT);

	if(strstr(dev->rxbuffer, RES_ACK)) return HAL_OK;
	return HAL_ERROR;
}

HAL_StatusTypeDef lidar_setRepIntFreq(lidar_t*, uint32_t) {
	 return HAL_OK;
}

HAL_StatusTypeDef lidar_meas(lidar_t* dev, uint32_t* res) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);



	sprintf()
	return HAL_OK;
}

HAL_StatusTypeDef lidar_start(lidar_t* dev) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);

	HAL_UART_Transmit(dev->huart, COM_START, strlen(COM_START), TX_TIMEOUT);

	HAL_UART_Receive(dev->huart, dev->rxbuffer, LIDAR_RXBUF_SIZE - 1, RX_TIMEOUT);

	if(strstr(dev->rxbuffer, RES_ACK)) return HAL_OK;
	return HAL_ERROR;
}

HAL_StatusTypeDef lidar_stop(lidar_t* dev) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);

	HAL_UART_Transmit(dev->huart, COM_STOP, strlen(COM_STOP), TX_TIMEOUT);

	HAL_UART_Receive(dev->huart, dev->rxbuffer, LIDAR_RXBUF_SIZE - 1, RX_TIMEOUT);

	if(strstr(dev->rxbuffer, RES_ACK)) return HAL_OK;
	return HAL_ERROR;
}
