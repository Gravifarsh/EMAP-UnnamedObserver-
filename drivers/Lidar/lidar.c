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
#define RES_METERS ("m")
#define RES_DISTANCE ("Distance:")

#define RX_TIMEOUT (15)
#define TX_TIMEOUT (100)

inline HAL_StatusTypeDef checkAck(lidar_t* dev) {
	char* ack = strstr(dev->rxbuffer, RES_ACK);

	if(ack == dev->rxbuffer || *(ack - 1) != 'N') return HAL_OK;
	return HAL_ERROR;
}

static HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while((__HAL_UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status)
  {
    /* Check for the Timeout */
    if(Timeout != HAL_MAX_DELAY)
    {
      if((Timeout == 0U)||((HAL_GetTick() - Tickstart ) > Timeout))
      {
        /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_TXEIE));
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        huart->gState  = HAL_UART_STATE_READY;
        huart->RxState = HAL_UART_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(huart);

        return HAL_TIMEOUT;
      }
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef Custom_HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	uint16_t* tmp;
	uint32_t tickstart = 0U;

	/* Check that a Rx process is not already ongoing */
	if(huart->RxState == HAL_UART_STATE_READY)
	{
	if((pData == NULL ) || (Size == 0U))
	{
	  return  HAL_ERROR;
	}

	/* Process Locked */
	__HAL_LOCK(huart);

	huart->ErrorCode = HAL_UART_ERROR_NONE;
	huart->RxState = HAL_UART_STATE_BUSY_RX;

	huart->RxXferSize = Size;
	huart->RxXferCount = Size;

	/* Check the remain data to be received */
	while(huart->RxXferCount > 0U)
	{
		/* Init tickstart for timeout managment */
			tickstart = HAL_GetTick();
	  huart->RxXferCount--;
	  if(huart->Init.WordLength == UART_WORDLENGTH_9B)
	  {
		if(UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_RXNE, RESET, tickstart, Timeout) != HAL_OK)
		{
		  return HAL_TIMEOUT;
		}
		tmp = (uint16_t*) pData;
		if(huart->Init.Parity == UART_PARITY_NONE)
		{
		  *tmp = (uint16_t)(huart->Instance->DR & (uint16_t)0x01FFU);
		  pData +=2U;
		}
		else
		{
		  *tmp = (uint16_t)(huart->Instance->DR & (uint16_t)0x00FFU);
		  pData +=1U;
		}

	  }
	  else
	  {
		if(UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_RXNE, RESET, tickstart, Timeout) != HAL_OK)
		{
		  return HAL_TIMEOUT;
		}
		if(huart->Init.Parity == UART_PARITY_NONE)
		{
		  *pData++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FFU);
		}
		else
		{
		  *pData++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x007FU);
		}

	  }
	}

	/* At end of Rx process, restore huart->RxState to Ready */
		huart->RxState = HAL_UART_STATE_READY;

	/* Process Unlocked */
		__HAL_UNLOCK(huart);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

HAL_StatusTypeDef lidar_tdcInit(lidar_t* dev) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);

	HAL_UART_Transmit(dev->huart, COM_TDCINIT, strlen(COM_TDCINIT), TX_TIMEOUT);

	HAL_UART_Receive(dev->huart, (uint8_t*)dev->rxbuffer, LIDAR_RXBUF_SIZE - 1, 10 * RX_TIMEOUT);

	return checkAck(dev);
}

HAL_StatusTypeDef lidar_setMeasFreq(lidar_t* dev, uint32_t freq) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);

	sprintf(dev->txbuffer, COM_MEASFREQ, freq);

	HAL_UART_Transmit(dev->huart, (uint8_t*)dev->txbuffer, strlen(dev->txbuffer), TX_TIMEOUT);

	HAL_UART_Receive(dev->huart, (uint8_t*)dev->rxbuffer, LIDAR_RXBUF_SIZE - 1, RX_TIMEOUT);

	return checkAck(dev);
}

HAL_StatusTypeDef lidar_setRepIntFreq(lidar_t* dev, uint32_t freq) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);

	sprintf(dev->txbuffer, COM_REPINTFREQ, freq);

	HAL_UART_Transmit(dev->huart, (uint8_t*)dev->txbuffer, strlen(dev->txbuffer), TX_TIMEOUT);

	HAL_UART_Receive(dev->huart, (uint8_t*)dev->rxbuffer, LIDAR_RXBUF_SIZE - 1, RX_TIMEOUT);

	if(checkAck(dev) == HAL_OK) {
		dev->repint_freq = freq;
		return HAL_OK;
	}
	else return HAL_ERROR;
}

HAL_StatusTypeDef lidar_meas(lidar_t* dev, uint32_t* res) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);

	HAL_UART_Transmit(dev->huart, COM_MEAS, strlen(COM_MEAS), TX_TIMEOUT);

	HAL_DMA_Start(dev->huart->hdmarx, (uint32_t)&dev->huart->Instance->DR,
			(uint32_t)dev->rxbuffer, LIDAR_RXBUF_SIZE - 1);

	return HAL_OK;
}

HAL_StatusTypeDef lidar_tryParseMeasRes(lidar_t* dev, uint32_t* res) {
	if(!strstr(dev->rxbuffer, RES_METERS)) return HAL_BUSY;

	char* dis = strstr(dev->rxbuffer, RES_DISTANCE);

	if(!dis) return HAL_ERROR;

	dis += strlen(RES_DISTANCE);

	float tmp = atoff(dis);
	*res = (uint32_t)(tmp * 1000);

	return HAL_OK;
}

HAL_StatusTypeDef lidar_dropMeas(lidar_t* dev) {
	return HAL_DMA_Abort(dev->huart->hdmarx);
}

HAL_StatusTypeDef lidar_start(lidar_t* dev) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);

	HAL_UART_Transmit(dev->huart, COM_START, strlen(COM_START), TX_TIMEOUT);

	HAL_UART_Receive(dev->huart, (uint8_t*)dev->rxbuffer, LIDAR_RXBUF_SIZE - 1, RX_TIMEOUT);

	return checkAck(dev);
}

HAL_StatusTypeDef lidar_stop(lidar_t* dev) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);

	HAL_UART_Transmit(dev->huart, COM_STOP, strlen(COM_STOP), TX_TIMEOUT);

	HAL_UART_Receive(dev->huart, (uint8_t*)dev->rxbuffer, LIDAR_RXBUF_SIZE - 1, RX_TIMEOUT);

	return checkAck(dev);
}

HAL_StatusTypeDef lidar_getRes(lidar_t* dev, uint32_t* min, uint32_t* mid, uint32_t* max) {
	memset(dev->rxbuffer, 0x00, LIDAR_RXBUF_SIZE);

	HAL_UART_Receive(dev->huart, (uint8_t*)dev->rxbuffer, LIDAR_RXBUF_SIZE - 1, dev->repint_freq);

	return HAL_OK;
}
