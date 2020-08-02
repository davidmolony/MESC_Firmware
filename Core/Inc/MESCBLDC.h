/*
 * MESCBLDC.h
 *
 *  Created on: 25 Jul 2020
 *      Author: Lenovo
 */
#include "stm32f3xx_hal.h"


#ifndef INC_MESCBLDC_H_
#define INC_MESCBLDC_H_


#endif /* INC_MESCBLDC_H_ */

typedef struct
{
float ReqCurrent;
int BLDCduty;
int BLDCEstate;
int CurrentChannel;
int pGain;
int iGain;
}MESCBLDCVars_s;

MESCBLDCVars_s BLDCVars;

typedef enum
{
	BLDC_FORWARDS=1,
	BLDC_BACKWARDS=2,
	BLDC_IDLE=3,
	BLDC_BRAKE=4
}MESCBLDCState_e;

MESCBLDCState_e BLDCState;


/* Function prototypes -----------------------------------------------*/
void BLDCInit();
void BLDCCommuteHall();
void BLDCCurrentController();
void writeBLDC();
