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
#include "stm32fxxx_hal.h"

#ifndef INC_MESCBLDC_H_
#define INC_MESCBLDC_H_

#endif /* INC_MESCBLDC_H_ */

#include "MESCfoc.h"

#define FOC_CONV_CHANNELS          (4)
#define FOC_TRANSFORMED_CHANNELS   (2)
#define FOC_NUM_ADC                (4)

typedef struct {
  int32_t RawADC[FOC_NUM_ADC]
                [FOC_CONV_CHANNELS];  // ADC1 returns Ucurrent, DClink
                                      // voltage and U phase voltage
                                      //  ADC2 returns Vcurrent, V and Wphase
                                      //  voltages
                                      // ADC3 returns Wcurrent
  // We can use ints rather than uints, since this later helps the conversion of
  // values to float, and the sign bit remains untouched (0)
  int32_t ADCOffset[FOC_NUM_ADC];  // During detect phase, need to sense the
                                   // zero current offset
  float ConvertedADC[FOC_NUM_ADC]
                    [FOC_CONV_CHANNELS];  // We will fill this with currents
                                          // in A and voltages in Volts
  uint32_t adc1, adc2, adc3, adc4, adc5;

} foc_measurement_t;

extern foc_measurement_t measurement_buffers;

typedef struct {
  float ReqCurrent;
  int BLDCduty;
  int BLDCEstate;
  int CurrentChannel;
  float currentCurrent;
  int pGain;
  int iGain;
} MESCBLDCVars_s;

extern MESCBLDCVars_s BLDCVars;

typedef enum {
  BLDC_FORWARDS = 1,
  BLDC_BACKWARDS = 2,
  BLDC_IDLE = 3,
  BLDC_BRAKE = 4
} MESCBLDCState_e;

extern MESCBLDCState_e BLDCState;

/* Function prototypes -----------------------------------------------*/
void BLDCInit();
void BLDCCommuteHall();
void BLDCCurrentController();
void writeBLDC();
int GetHallState();  // Self explanatory...
void BLDCCommute(MESC_motor_typedef *_motor);
void CalculateBLDCGains(MESC_motor_typedef *_motor);
