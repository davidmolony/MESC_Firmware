/*
 **
 ******************************************************************************
 * @file           : MESCBLDC.h
 * @brief          : BLDC running code
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 David Molony.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************

 * MESCBLDC.h
 *
 *  Created on: 25 Jul 2020
 *      Author: David Molony
 */
#include "stm32f3xx_hal.h"

#ifndef INC_MESCBLDC_H_
#define INC_MESCBLDC_H_

#endif /* INC_MESCBLDC_H_ */

typedef struct {
	float ReqCurrent;
	int BLDCduty;
	int BLDCEstate;
	int CurrentChannel;
	float currentCurrent;
	int pGain;
	int iGain;
} MESCBLDCVars_s;

MESCBLDCVars_s BLDCVars;

typedef enum {
	BLDC_FORWARDS = 1, BLDC_BACKWARDS = 2, BLDC_IDLE = 3, BLDC_BRAKE = 4
} MESCBLDCState_e;

MESCBLDCState_e BLDCState;

/* Function prototypes -----------------------------------------------*/
void BLDCInit();
void BLDCCommuteHall();
void BLDCCurrentController();
void writeBLDC();
