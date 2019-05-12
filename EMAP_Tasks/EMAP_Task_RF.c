/*
 * EMAP_Task_RF.c
 *
 *  Created on: 20 апр. 2019 г.
 *      Author: developer
 */

#include "EMAP_Task_RF.h"

#include "FreeRTOS.h"
#include "task.h"

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
	spi_nRF24L01.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	spi_nRF24L01.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi_nRF24L01.Init.TIMode = SPI_TIMODE_DISABLE;
	spi_nRF24L01.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi_nRF24L01.Init.CRCPolynomial = 10;

	HAL_SPI_Init(&spi_nRF24L01);
}

void send(uint8_t* data, size_t size) {
	size_t pos = 0;

	while(pos < size) {
		nRF24.PayloadWidth = (size - pos) > 32 ? 32 : size - pos;
		HAL_nRF24L01P_TransmitPacketNonExt(&nRF24, data + pos);
		pos += 32;
	}
}

void RF_Task() {
	uint8_t buffer[RF_TX_BUFFER_SIZE];
	for(;;) {
		trace_puts("RF TASK");
		vTaskDelay(10 / portTICK_RATE_MS);


		buffer[0] = 0xFA;
taskENTER_CRITICAL();
		memcpy(buffer + 1, &data_BMP280_1, sizeof(data_BMP280_1));
taskEXIT_CRITICAL();
		buffer[sizeof(data_BMP280_1) + 1] = 0xFA;

		send(buffer, sizeof(data_BMP280_1) + 2);

		buffer[0] = 0xFB;
taskENTER_CRITICAL();
		memcpy(buffer + 1, &data_BMP280_2, sizeof(data_BMP280_2));
taskEXIT_CRITICAL();
		buffer[sizeof(data_BMP280_2) + 1] = 0xFB;

		send(buffer, sizeof(data_BMP280_2) + 2);

		buffer[0] = 0xFC;
taskENTER_CRITICAL();
		memcpy(buffer + 1, &data_MPU9255_1, sizeof(data_MPU9255_1));
taskEXIT_CRITICAL();
		buffer[sizeof(data_MPU9255_1) + 1] = 0xFC;

		send(buffer, sizeof(data_MPU9255_1) + 2);

		buffer[0] = 0xFB;
taskENTER_CRITICAL();
		memcpy(buffer + 1, &data_MPU9255_2, sizeof(data_MPU9255_2));
taskEXIT_CRITICAL();
		buffer[sizeof(data_MPU9255_2) + 1] = 0xFB;

		send(buffer, sizeof(data_MPU9255_2) + 2);

		buffer[0] = 0xFD;
taskENTER_CRITICAL();
		memcpy(buffer + 1, &system_state, sizeof(system_state));
taskEXIT_CRITICAL();
		buffer[sizeof(system_state) + 1] = 0xFD;

		send(buffer, sizeof(system_state) + 2);
	}
}

void nRF_Init(){
	GPIO_Init();
	SPI_Init();

	nRF24.hspi = &spi_nRF24L01;
	nRF24.CRC_Width = nRF_CRC_WIDTH_BYTE;
	nRF24.ADDR_Width = nRF_ADDR_WIDTH_5;
	nRF24.Data_Rate = nRF_DATA_RATE_1MBPS;
	nRF24.TX_Power = nRF_TX_PWR_0dBm;
	nRF24.State = nRF_STATE_TX;

	nRF24.RF_Channel = 73;
	nRF24.PayloadWidth = nRF_RXPW_32BYTES;
	nRF24.RetransmitCount = nRF_RETX_COUNT_15;

	nRF24.RetransmitDelay = nRF_RETX_DELAY_1000uS;

	nRF24.RX_Address = (uint8_t *)RXAddr;
	nRF24.TX_Address = (uint8_t *)TXAddr;

	nRF24.RX_Buffer = RXBuffer;

	nRF24.nRF_nSS_GPIO_PORT = GPIOA;
	nRF24.nRF_nSS_GPIO_PIN = GPIO_PIN_4;
	nRF24.nRF_CE_GPIO_PORT = GPIOA;
	nRF24.nRF_CE_GPIO_PIN = GPIO_PIN_3;

	trace_printf("nRF: %d\n", HAL_nRF24L01P_Init(&nRF24));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_2) {
		HAL_nRF24L01P_IRQ_Handler(&nRF24);
	}
}

void EXTI2_IRQHandler(){
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}
