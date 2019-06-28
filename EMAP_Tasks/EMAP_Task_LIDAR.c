/*
 * EMAP_Task_LIDAR.c
 *
 *  Created on: 25 июн. 2019 г.
 *      Author: developer
 */


#include "EMAP_Task_LIDAR.h"

UART_HandleTypeDef	uart_lidar;
DMA_HandleTypeDef	dma_lidar;
lidar_t				lidar;

void LIDAR_Task() {
	uint32_t dummy;
	for(;;) {
		trace_printf("%d\n", lidar_meas(&lidar));

		while(lidar_tryParseMeasRes(&lidar, &dummy) != HAL_OK) {
			trace_printf("%s\n\n\n", lidar.rxbuffer);
		}

		trace_printf("%d\n", lidar_dropMeas(&lidar));
		trace_printf("Distance: %d\n", dummy);
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

	__HAL_RCC_DMA1_CLK_ENABLE();

	dma_lidar.Instance = DMA1_Stream2;
	dma_lidar.Init.Channel = DMA_CHANNEL_4;						// 4 канал - на USART1_RX
	dma_lidar.Init.Direction = DMA_PERIPH_TO_MEMORY;				// направление - из периферии в память
	dma_lidar.Init.PeriphInc = DMA_PINC_DISABLE;					// инкрементация периферии выключена
	dma_lidar.Init.MemInc = DMA_MINC_ENABLE;						// инкрементация памяти включена
	dma_lidar.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;		// длина слова в периферии - байт
	dma_lidar.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;		// длина слова в памяти - байт
	dma_lidar.Init.Mode = DMA_NORMAL;							// режим - обычный
	dma_lidar.Init.Priority = DMA_PRIORITY_HIGH;					// приоритет - средний
	dma_lidar.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	dma_lidar.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	dma_lidar.Init.MemBurst = DMA_MBURST_SINGLE;
	dma_lidar.Init.PeriphBurst = DMA_PBURST_SINGLE;

	HAL_DMA_Init(&dma_lidar);

	lidar.huart->hdmarx = &dma_lidar;

	/* Enable the DMA transfer for the receiver request by setting the DMAR bit
	in the UART CR3 register */
	SET_BIT(uart_lidar.Instance->CR3, USART_CR3_DMAR);

	trace_printf("LIDAR: %d\n", lidar_tdcInit(&lidar));
}
