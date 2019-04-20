/*
 * EMAP_Task_RF.c
 *
 *  Created on: 20 апр. 2019 г.
 *      Author: developer
 */

#include "EMAP_Task_RF.h"

#include "diag/Trace.h"

const uint8_t RXAddr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
const uint8_t TXAddr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

uint8_t RXBuffer[32];

SPI_HandleTypeDef	spi_nRF24L01;
nRF24L01P 	nRF24;

void GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

void SPI_Init(void)
{
	spi_nRF24L01.Instance = SPI1;
	spi_nRF24L01.Init.Mode = SPI_MODE_MASTER;
	spi_nRF24L01.Init.Direction = SPI_DIRECTION_2LINES;
	spi_nRF24L01.Init.DataSize = SPI_DATASIZE_8BIT;
	spi_nRF24L01.Init.CLKPolarity = SPI_POLARITY_LOW;
	spi_nRF24L01.Init.CLKPhase = SPI_PHASE_1EDGE;
	spi_nRF24L01.Init.NSS = SPI_NSS_SOFT;
	spi_nRF24L01.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	spi_nRF24L01.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi_nRF24L01.Init.TIMode = SPI_TIMODE_DISABLE;
	spi_nRF24L01.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi_nRF24L01.Init.CRCPolynomial = 10;

	HAL_SPI_Init(&spi_nRF24L01);
}

void nRF_Init(){
	GPIO_Init();
	SPI_Init();

	nRF24.hspi = &spi_nRF24L01;
	nRF24.CRC_Width = nRF_CRC_WIDTH_BYTE;
	nRF24.ADDR_Width = nRF_ADDR_WIDTH_5;
	nRF24.Data_Rate = nRF_DATA_RATE_2MBPS;
	nRF24.TX_Power = nRF_TX_PWR_0dBm;
	nRF24.State = nRF_STATE_TX;

	nRF24.RF_Channel = 2;
	nRF24.PayloadWidth = nRF_RXPW_32BYTES;
	nRF24.RetransmitCount = nRF_RETX_DISABLED;
	nRF24.RetransmitDelay = nRF_RETX_DELAY_1000uS;

	nRF24.RX_Address = (uint8_t *)RXAddr;
	nRF24.TX_Address = (uint8_t *)TXAddr;

	nRF24.RX_Buffer = RXBuffer;

	nRF24.nRF_nSS_GPIO_PORT = GPIOA;
	nRF24.nRF_nSS_GPIO_PIN = GPIO_PIN_4;
	nRF24.nRF_CE_GPIO_PORT = GPIOA;
	nRF24.nRF_CE_GPIO_PIN = GPIO_PIN_3;

	if(HAL_nRF24L01P_Init(&nRF24) != HAL_OK)
		trace_printf("Fail nRF init\n");
	else
		trace_printf("Ok nRF init\n");
}
