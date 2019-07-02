/*
 * EMAP_Task_SD.c
 *
 *  Created on: 19 апр. 2019 г.
 *      Author: developer
 */

#include "DATA_Helper.h"

#include "dump.h"

/* FOR SD */

static dump_channel_state_t stream_file;
uint8_t buf[BLOCK_SIZE];
uint32_t pos = 0;
bool isZeroWrote = false;

/* FOR NRF */

const uint8_t RXAddr[5] = {0x14, 0x14, 0x14, 0x14, 0x14};
const uint8_t TXAddr[5] = {0x14, 0x14, 0x14, 0x14, 0x14};

uint8_t RXBuffer[32];

SPI_HandleTypeDef	spi_nRF24L01;
nRF24L01P 			nRF24;

SemaphoreHandle_t spi_semphr;

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
		HAL_nRF24L01P_TransmitPacketNonExt(&nRF24, data + pos, NRF_TIMEOUT);
		pos += 32;
	}
}

uint8_t hash(uint8_t* data, size_t size) {
	uint8_t retval = 0;
	for(int i = 0; i < size; i++)
		retval = retval ^ data[i];
	return retval;
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

	nRF24.RF_Channel = 0;
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

	spi_semphr = xSemaphoreCreateMutex();
}

void writeSysStateZero()
{
	uint8_t buffer[sizeof(system_state_zero_t) + 3];
//taskENTER_CRITICAL();

	buffer[0] = 0xF9;
	buffer[1] = hash((uint8_t*)&system_state_zero, sizeof(system_state_zero_t));
	memcpy(buffer + 2, &system_state_zero, sizeof(system_state_zero_t));
	buffer[sizeof(system_state_zero_t) + 2] = 0xF9;

	while(xSemaphoreTake(spi_semphr, SEM_WAIT_MS  / portTICK_RATE_MS) != pdTRUE) {}
	drop(buffer, sizeof(buffer));
	xSemaphoreGive(spi_semphr);
//taskEXIT_CRITICAL();
}

void writeSysState()
{

	uint8_t buffer[sizeof(system_state_t) + 3];

//taskENTER_CRITICAL();
	system_state.SD = stream_file.res;

	buffer[0] = 0xFF;
	buffer[1] = hash((uint8_t*)&system_state, sizeof(system_state_t));
	memcpy(buffer + 2, &system_state, sizeof(system_state_t));
	buffer[sizeof(system_state_t) + 2] = 0xFF;

	while(xSemaphoreTake(spi_semphr, SEM_WAIT_MS / portTICK_RATE_MS) != pdTRUE) {}
	drop(buffer, sizeof(buffer));
	send(buffer, sizeof(buffer));
	xSemaphoreGive(spi_semphr);

//taskEXIT_CRITICAL();
}

#define TEST_TIME (10)

void writeDataMPU(I2C_HandleTypeDef * hi2c)
{
	uint8_t buffer[sizeof(data_MPU9255_t) + 3];

//taskENTER_CRITICAL();
	if(hi2c->Instance == I2C1)
	{
		buffer[0] = 0xFA;
		buffer[1] = hash((uint8_t*)&data_MPU9255_1, sizeof(data_MPU9255_t));
		memcpy(buffer + 2, &data_MPU9255_1, sizeof(data_MPU9255_t));
		buffer[sizeof(data_MPU9255_t) + 2] = 0xFA;
	}
	else
	{
		buffer[0] = 0xFB;
		buffer[1] = hash((uint8_t*)&data_MPU9255_2, sizeof(data_MPU9255_t));
		memcpy(buffer + 2, &data_MPU9255_2, sizeof(data_MPU9255_t));
		buffer[sizeof(data_MPU9255_t) + 2] = 0xFB;
	}

	while(xSemaphoreTake(spi_semphr, SEM_WAIT_MS / portTICK_RATE_MS) != pdTRUE) {}
	drop(buffer, sizeof(buffer));
	send(buffer, sizeof(buffer));
	xSemaphoreGive(spi_semphr);

//taskEXIT_CRITICAL();
}

void writeDataIsc()
{
	uint8_t buffer[sizeof(data_MPU9255_t) + 3];

//taskENTER_CRITICAL();
	buffer[0] = 0xFC;
	buffer[1] = hash((uint8_t*)&data_MPU9255_isc, sizeof(data_MPU9255_t));
	memcpy(buffer + 2, &data_MPU9255_isc, sizeof(data_MPU9255_t));
	buffer[sizeof(data_MPU9255_t) + 2] = 0xFC;

	while(xSemaphoreTake(spi_semphr, SEM_WAIT_MS / portTICK_RATE_MS) == pdTRUE) {}
	drop(buffer, sizeof(buffer));
	send(buffer, sizeof(buffer));
	xSemaphoreGive(spi_semphr);

//taskEXIT_CRITICAL();
}

void writeDataBMP()
{
	uint8_t buffer[sizeof(data_BMP280_t) + 3];

//taskENTER_CRITICAL();
	buffer[0] = 0xFD;
	buffer[1] = hash((uint8_t*)&data_BMP280_1, sizeof(data_BMP280_t));
	memcpy(buffer + 2, &data_BMP280_1, sizeof(data_BMP280_t));
	buffer[sizeof(data_BMP280_t) + 2] = 0xFD;

	while(xSemaphoreTake(spi_semphr, SEM_WAIT_MS / portTICK_RATE_MS) != pdTRUE) {}
	drop(buffer, sizeof(buffer));
	send(buffer, sizeof(buffer));
	xSemaphoreGive(spi_semphr);

	buffer[0] = 0xFE;
	buffer[1] = hash((uint8_t*)&data_BMP280_2, sizeof(data_BMP280_t));
	memcpy(buffer + 2, &data_BMP280_2, sizeof(data_BMP280_t));
	buffer[sizeof(data_BMP280_t) + 2] = 0xFE;

	while(xSemaphoreTake(spi_semphr, SEM_WAIT_MS / portTICK_RATE_MS) != pdTRUE) {}
	drop(buffer, sizeof(buffer));
	send(buffer, sizeof(buffer));
	xSemaphoreGive(spi_semphr);
//taskEXIT_CRITICAL();
}

void writeDataTSL()
{
	uint8_t buffer[sizeof(data_TSL_t) + 3];

//taskENTER_CRITICAL();
	buffer[0] = 0xF8;
	buffer[1] = hash((uint8_t*)&data_TSL, sizeof(data_TSL_t));
	memcpy(buffer + 2, &data_TSL, sizeof(data_TSL_t));
	buffer[sizeof(data_TSL_t) + 2] = 0xF8;

	while(xSemaphoreTake(spi_semphr, SEM_WAIT_MS / portTICK_RATE_MS) != pdTRUE) {}
	drop(buffer, sizeof(buffer));
	send(buffer, sizeof(buffer));
	xSemaphoreGive(spi_semphr);
//taskEXIT_CRITICAL();
}

void writeDataLidar()
{
	uint8_t buffer[sizeof(data_LIDAR_t) + 3];

//taskENTER_CRITICAL();
	buffer[0] = 0xF7;
	buffer[1] = hash((uint8_t*)&data_LIDAR, sizeof(data_LIDAR_t));
	memcpy(buffer + 2, &data_LIDAR, sizeof(data_LIDAR_t));
	buffer[sizeof(data_LIDAR_t) + 2] = 0xF7;

	while(xSemaphoreTake(spi_semphr, SEM_WAIT_MS / portTICK_RATE_MS) != pdTRUE) {}
	drop(buffer, sizeof(buffer));
	send(buffer, sizeof(buffer));
	xSemaphoreGive(spi_semphr);
//taskEXIT_CRITICAL();
}

void writeDataGPS()
{
	uint8_t buffer[sizeof(data_GPS_t) + 3];

//taskENTER_CRITICAL();
	buffer[0] = 0xF6;
	buffer[1] = hash((uint8_t*)&data_GPS, sizeof(data_GPS_t));
	memcpy(buffer + 1, &data_GPS, sizeof(data_GPS_t));
	buffer[sizeof(data_GPS_t) + 2] = 0xF6;

	while(xSemaphoreTake(spi_semphr, SEM_WAIT_MS / portTICK_RATE_MS) != pdTRUE) {}
	drop(buffer, sizeof(buffer));
	send(buffer, sizeof(buffer));
	xSemaphoreGive(spi_semphr);
//taskEXIT_CRITICAL();
}
