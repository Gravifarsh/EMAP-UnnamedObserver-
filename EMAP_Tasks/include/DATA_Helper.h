/*
 * EMAP_Task_SD.h
 *
 *  Created on: 19 апр. 2019 г.
 *      Author: developer
 */

#ifndef INCLUDE_EMAP_TASK_SD_H_
#define INCLUDE_EMAP_TASK_SD_H_

#include "EMAPConfig.h"

#define BLOCK_SIZE	512

void DATA_Init();

void writeDataBMP();
void writeDataIsc();
void writeDataMPU(I2C_HandleTypeDef * hi2c);
void writeSysState();
void writeSysStateZero();
void writeDataTSL();
void writeDataLidar();

#endif /* INCLUDE_EMAP_TASK_SD_H_ */
