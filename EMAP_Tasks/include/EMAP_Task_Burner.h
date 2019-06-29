/*
 * EMAP_Task_Burner.h
 *
 *  Created on: 29 июн. 2019 г.
 *      Author: developer
 */

#ifndef INCLUDE_EMAP_TASK_BURNER_H_
#define INCLUDE_EMAP_TASK_BURNER_H_

#include "EMAPConfig.h"

#define PAYLOAD_TIME 10000
#define TSL_DARK	 150
#define TSL_BRIGHT	 10000
#define FALLING_TIME 2000

void BURNER_Init();
void BURNER_Task();

#endif /* INCLUDE_EMAP_TASK_BURNER_H_ */
