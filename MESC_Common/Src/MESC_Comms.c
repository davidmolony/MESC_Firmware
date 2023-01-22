/*
 **
 ******************************************************************************
 * @file           : MESC_Comms.c
 * @brief          : UART parsing and RCPWM input
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

 * MESC_Comms.c
 *
 *  Created on: 15 Nov 2020
 *      Author: David Molony
 */

/* Includes ------------------------------------------------------------------*/

#include "MESC_Comms.h"

#include <stdio.h>

#include "MESCBLDC.h"
#include "MESCfoc.h"
#include "MESChw_setup.h"
#include "MESCmotor_state.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////
// RCPWM implementation
//////////////////////////////////////////////////////////////////////////////////////////////////////
/*

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  uint16_t ICVals[2];
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    ICVals[0] = HAL_TIM_ReadCapturedValue(htim &htim3, TIM_CHANNEL_1);

    // Target is 20000 guard is +-10000
    if ((ICVals[0] < 10000) || (30000 < ICVals[0])) {
      BLDCVars.ReqCurrent = 0;
      foc_vars.Idq_req.d = 0;
      foc_vars.Idq_req.q = 0;
    }

    else if (ICVals[0] != 0) {
      BLDCState = BLDC_FORWARDS;
      ICVals[1] = HAL_TIM_ReadCapturedValue(htim &htim3, TIM_CHANNEL_2);
      if (ICVals[1] > 2000) ICVals[1] = 2000;
      if (ICVals[1] < 1000) ICVals[1] = 1000;

      // Mid-point is 1500 guard is +-100
      if ((ICVals[1] > 1400) && (1600 > ICVals[1])) {
        ICVals[1] = 1500;
      }
      // Set the current setpoint here
      if (1) {  // Current control, ToDo convert to Enum
        if (ICVals[1] > 1600) {
          BLDCVars.ReqCurrent = ((float)ICVals[1] - 1600) / 5.0f;
          foc_vars.Idq_req.d = 0;
          foc_vars.Idq_req.q = ((float)ICVals[1] - 1600) / 5.0f;
        }
        // Crude hack, which gets current scaled to +/-80A
        // based on 1000-2000us PWM in
        else if (ICVals[1] < 1400) {
          BLDCVars.ReqCurrent = ((float)ICVals[1] - 1600) / 5.0f;
          foc_vars.Idq_req.d = 0;
          foc_vars.Idq_req.q = ((float)ICVals[1] - 1400) / 5.0f;
        }
        // Crude hack, which gets current scaled to +/-80A
        // based on 1000-2000us PWM in
        else {
          static float currentset = 0;
          BLDCVars.ReqCurrent = 0;
          foc_vars.Idq_req.d = 0;
          foc_vars.Idq_req.q = currentset;
        }
      }
    }
  }
}
*/
