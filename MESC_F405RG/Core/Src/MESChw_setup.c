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

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

void motor_init() {
  motor.Rphase = 0;  // We init at 0 to trigger the measurer to get the vals
  motor.Lphase = 0;  // We init at 0 to trigger the measurer to get the vals
  motor.uncertainty = 1;
}

void hw_init() {
  g_hw_setup.Imax =
      100.0;  // Imax is the current at which we are either no longer able to
             // read it, or hardware "don't ever exceed to avoid breakage"
  g_hw_setup.Vmax = 55.0;  // Headroom beyond which likely to get avalanche of
                           // MOSFETs or DCDC converter
  g_hw_setup.Vmin = 10;  // This implies that the PSU has crapped out or a wire
                         // has fallen out, and suddenly there will be no power.
  g_hw_setup.Rshunt = 0.00033;
  g_hw_setup.RVBB = 1500;   //
  g_hw_setup.RVBT = 82000;  //
  g_hw_setup.OpGain = 10;   //
  g_hw_setup.VBGain =
      (3.3f / 4096.0f) * (g_hw_setup.RVBB + g_hw_setup.RVBT) / g_hw_setup.RVBB;
  g_hw_setup.Igain = 3.3 / (g_hw_setup.Rshunt * 4096 * g_hw_setup.OpGain * SHUNT_POLARITY);  // TODO
  g_hw_setup.RawCurrLim =
      g_hw_setup.Imax * g_hw_setup.Rshunt * g_hw_setup.OpGain * (4096 / 3.3) +
      2048;
  if (g_hw_setup.RawCurrLim > 4000) {
    g_hw_setup.RawCurrLim = 4000;
  }  // 4000 is 96 counts away from ADC saturation, allow headroom for opamp not
     // pulling rail:rail.
  g_hw_setup.RawVoltLim =
      (uint16_t)(4096.0f * (g_hw_setup.Vmax / 3.3f) * g_hw_setup.RVBB /
                 (g_hw_setup.RVBB + g_hw_setup.RVBT));
  g_hw_setup.battMaxPower = 500.0f;
}

void getRawADC(void) {
  measurement_buffers.RawADC[0][0] = hadc1.Instance->JDR1;  // U Current
  measurement_buffers.RawADC[0][1] = hadc3.Instance->JDR2;  // DC Link Voltage

  measurement_buffers.RawADC[1][0] = hadc2.Instance->JDR1;  // V Current
  measurement_buffers.RawADC[2][0] = hadc3.Instance->JDR1;  // W Current
  measurement_buffers.RawADC[1][3] = hadc1.Instance->JDR3;  // Throttle

  measurement_buffers.RawADC[0][2] = hadc1.Instance->JDR2;//PhaseU Voltage
  measurement_buffers.RawADC[1][1] = hadc2.Instance->JDR3;//PhaseV Voltage
  measurement_buffers.RawADC[1][2] = hadc3.Instance->JDR3;//PhaseW Voltage
}

void getRawADCVph(void){



}
