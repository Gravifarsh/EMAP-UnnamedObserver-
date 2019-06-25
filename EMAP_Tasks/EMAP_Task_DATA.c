/*
 * EMAP_Task_SD.c
 *
 *  Created on: 19 апр. 2019 г.
 *      Author: developer
 */

#include "stdint.h"
#include "inttypes.h"

#include "diag/Trace.h"
#include "FreeRTOS.h"
#include "task.h"

#include "dump.h"

#include "EMAP_Task_DATA.h"

/* FOR SD */

static dump_channel_state_t stream_file;
uint8_t buf[BLOCK_SIZE];
uint32_t pos = 0;
bool isZeroWrote = false;

/* FOR NRF */

const uint8_t RXAddr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
const uint8_t TXAddr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

uint8_t RXBuffer[32];

SPI_HandleTypeDef	spi_nRF24L01;
nRF24L01P 			nRF24;

void drop(const void * data, size_t datasize)
{
	size_t datapos = 0;

	while(datapos < datasize) {
		buf[pos++] = *((uint8_t*)data + datapos++);

		if(pos == BLOCK_SIZE) {
			dump(&stream_file, buf, BLOCK_SIZE);
			pos = 0;
		}
	}
}

void send(uint8_t* data, size_t size) {
	size_t pos = 0;

	while(pos < size) {
		nRF24.PayloadWidth = (size - pos) > 32 ? 32 : size - pos;
		HAL_nRF24L01P_TransmitPacketNonExt(&nRF24, data + pos);
		pos += 32;
	}
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

void nRF_Init(){
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

void SD_Init()
{
	memset(&stream_file, 0x00, sizeof(stream_file));

	dump_init(&stream_file);
	system_state.SD = (uint8_t)stream_file.res;
}

void DATA_Init() {
	SPI_Init();
	nRF_Init();
	SD_Init();
}

void writeSysStateZero()
{
	uint8_t buffer[sizeof(system_state_zero_t) + 2];

	buffer[0] = 0xF9;
	memcpy(&buffer[2], &system_state_zero, sizeof(system_state_zero_t));
	buffer[sizeof(system_state_zero_t) + 1] = 0xF9;

	drop(buffer, sizeof(system_state_zero_t) + 2);
}

void writeSysState()
{

	uint8_t buffer[sizeof(system_state_t) + 2];

taskENTER_CRITICAL();
	system_state.SD = stream_file.res;

	buffer[0] = 0xFF;
	memcpy(buffer + 1, &system_state, sizeof(system_state_t));
	buffer[sizeof(system_state_t) + 1] = 0xFF;
taskEXIT_CRITICAL();

	drop(buffer, sizeof(system_state_t) + 2);
	send(buffer, sizeof(system_state_t) + 2);
}

void writeDataMPU(I2C_HandleTypeDef * hi2c)
{
	uint8_t buffer[sizeof(data_MPU9255_t) + 2];

taskENTER_CRITICAL();
	if(hi2c->Instance == I2C1)
	{
		buffer[0] = 0xFA;
		memcpy(buffer + 1, &data_MPU9255_1, sizeof(data_MPU9255_t));
		buffer[sizeof(data_MPU9255_1) + 1] = 0xFA;
	}
	else
	{
		buffer[0] = 0xFB;
		memcpy(buffer + 1, &data_MPU9255_2, sizeof(data_MPU9255_t));
		buffer[sizeof(data_MPU9255_2) + 1] = 0xFB;
	}
taskEXIT_CRITICAL();

	drop(buffer, sizeof(data_MPU9255_t) + 2);
	send(buffer, sizeof(data_MPU9255_t) + 2);
}

void writeDataIsc()
{
	uint8_t buffer[sizeof(data_MPU9255_t) + 2];

taskENTER_CRITICAL();
	buffer[0] = 0xFC;
	memcpy(buffer + 1, &data_MPU9255_isc, sizeof(data_MPU9255_t));
	buffer[sizeof(data_MPU9255_t) + 1] = 0xFC;
taskEXIT_CRITICAL();

	drop(buffer, sizeof(data_MPU9255_t) + 2);
	send(buffer, sizeof(data_MPU9255_t) + 2);
}

void writeDataBMP()
{
	uint8_t buffer[sizeof(data_BMP280_t) + 2];

taskENTER_CRITICAL();
	buffer[0] = 0xFD;
	memcpy(buffer + 1, &data_BMP280_1, sizeof(data_BMP280_t));
	buffer[sizeof(data_BMP280_t) + 1] = 0xFD;
taskEXIT_CRITICAL();

	drop(buffer, sizeof(data_BMP280_t) + 2);
	send(buffer, sizeof(data_BMP280_t) + 2);

taskENTER_CRITICAL();
	buffer[0] = 0xFE;
	memcpy(buffer + 1, &data_BMP280_2, sizeof(data_BMP280_t));
	buffer[sizeof(data_BMP280_t) + 1] = 0xFE;
taskEXIT_CRITICAL();

	drop(buffer, sizeof(data_BMP280_t) + 2);
	send(buffer, sizeof(data_BMP280_t) + 2);
}

void DATA_Task()
{
	if(stream_file.file_opened == false)
	{
		trace_printf("SD_Task Shut Down!\n");
		vTaskDelete(NULL);
	}

	trace_printf("SD State: %d\n", system_state.SD);


	uint32_t time = HAL_GetTick();
	for(;;)
	{
		trace_printf("DATA TASK TIME ELAPSED: %d\n", HAL_GetTick() - time);
		time = HAL_GetTick();

		if(isZeroWrote)
		{
			writeSysState();
			writeDataMPU(&i2c_IMU_1);
			writeDataMPU(&i2c_IMU_2);
			writeDataIsc();
			writeDataBMP();
		}
		else
		{
			if(system_state_zero.pressure)
			{
				writeSysStateZero();
				isZeroWrote = true;
			}
		}
	}
}


