/*
 **
 ******************************************************************************
 * @file           : MESChw_setup.c
 * @brief          : Initialisation code for the PCB
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

 * MESChw_setup.c
 *
 *  Created on: 25 Jul 2020
 *      Author: David Molony
 */
/* Includes ------------------------------------------------------------------*/
#include "MESChw_setup.h"

#include "MESCfoc.h"
#include "MESCpwm.h"

extern ADC_HandleTypeDef hadc1;

extern TIM_HandleTypeDef htim1;

hw_setup_s g_hw_setup;
motor_s motor;

uint32_t ADC_buffer[6];

void hw_init(MESC_motor_typedef *_motor) {
  g_hw_setup.Imax = ABS_MAX_PHASE_CURRENT;  	// Imax is the current at which we are either no longer able to
             	 	 	 	 	 	 	 	 	// read it, or hardware "don't ever exceed to avoid breakage"
  g_hw_setup.Vmax = ABS_MAX_BUS_VOLTAGE;  // Headroom beyond which likely to get avalanche of
                           	   	   	   	  // MOSFETs or DCDC converter
  g_hw_setup.Vmin = ABS_MIN_BUS_VOLTAGE;  // This implies that the PSU has crapped out or a wire
                         // has fallen out, and suddenly there will be no power.
  g_hw_setup.Rshunt = R_SHUNT;
  g_hw_setup.RVBB = R_VBUS_BOTTOM;   //
  g_hw_setup.RVBT = R_VBUS_TOP;  //
  g_hw_setup.OpGain = OPGAIN;   //
  g_hw_setup.VBGain =
      (3.3f / 4096.0f) * (g_hw_setup.RVBB + g_hw_setup.RVBT) / g_hw_setup.RVBB;
  g_hw_setup.Igain = 3.3f / (g_hw_setup.Rshunt * 4096 * g_hw_setup.OpGain * SHUNT_POLARITY);  // TODO
  g_hw_setup.RawCurrLim =
      g_hw_setup.Imax * g_hw_setup.Rshunt * g_hw_setup.OpGain * (4096 / 3.3f) +
      2048;
  if (g_hw_setup.RawCurrLim > 4000) {
    g_hw_setup.RawCurrLim = 4000;
  }  // 4000 is 96 counts away from ADC saturation, allow headroom for opamp not
     // pulling rail:rail.
  g_hw_setup.RawVoltLim =
      (uint16_t)(4096.0f * (g_hw_setup.Vmax / 3.3f) * g_hw_setup.RVBB /
                 (g_hw_setup.RVBB + g_hw_setup.RVBT));
}

void getRawADC(MESC_motor_typedef *_motor) {
//Get the injected critical conversions
  _motor->Raw.Iu = hadc1.Instance->JDR1;// PhaseU Current
  _motor->Raw.Iv = hadc1.Instance->JDR2;
  _motor->Raw.Iw = hadc1.Instance->JDR3;
  _motor->Raw.Vbus = hadc1.Instance->JDR4; //Bus/battery voltage

//These are handled by regular conversion manager and DMA
  GET_THROTTLE_INPUT;

//MOS temperature for MP2
  _motor->Raw.MOSu_T = ADC_buffer[5]; //Temperature on PB1
//Motor temp or Brake, needs plumbing in to main MESC...
  _motor->Raw.ADC_in_ext2 = ADC_buffer[4];
}

void getRawADCVph(MESC_motor_typedef *_motor){
	//Voltage sense for the MP2
	  _motor->Raw.Vu = ADC_buffer[0]; //PhaseU Voltage
	  _motor->Raw.Vv = ADC_buffer[1];
	  _motor->Raw.Vw = ADC_buffer[2];
}



void mesc_init_1( MESC_motor_typedef *_motor )
{
    // Do nothing
}

void mesc_init_2( MESC_motor_typedef *_motor )
{
    // Do nothing
}

void mesc_init_3( MESC_motor_typedef *_motor )
{
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_buffer, 6);
	HAL_ADCEx_InjectedStart(&hadc1);

	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_3);
	MESCpwm_generateBreak(_motor); //avoid a spurious pulse on startup
	HAL_Delay(10); //Delay enabling interrupt to avoid spurious error on startup due to ADC not being ready

	__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD); //ToDo, how do I put this into the whole shabang with multiple motors?...
	__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
	__HAL_TIM_ENABLE_IT(_motor->mtimer, TIM_IT_UPDATE);

	//Set up the input capture for throttle
	HAL_TIM_IC_Start(_motor->stimer, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(_motor->stimer, TIM_CHANNEL_2);
	__HAL_TIM_ENABLE_IT(_motor->stimer, TIM_IT_UPDATE);
	// Here we can auto set the prescaler to get the us input regardless of the main clock
	__HAL_TIM_SET_PRESCALER(_motor->stimer, (HAL_RCC_GetHCLKFreq() / 1000000 - 1));
}
