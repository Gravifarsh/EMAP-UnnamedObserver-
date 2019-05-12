/*
 * EMAP_Task_RF.h
 *
 *  Created on: 20 апр. 2019 г.
 *      Author: developer
 */

#ifndef INCLUDE_EMAP_TASK_RF_H_
#define INCLUDE_EMAP_TASK_RF_H_

#include "EMAPConfig.h"

#define RF_TX_BUFFER_SIZE (100)

void nRF_Init();
void RF_Task();

#endif /* INCLUDE_EMAP_TASK_RF_H_ */
