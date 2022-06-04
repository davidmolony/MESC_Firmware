/*
 **
 ******************************************************************************
 * @file           : MESC_Comms.h
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

 * MESC_Comms.h
 *
 *  Created on: 15 Nov 2020
 *      Author: David Molony
 */

#include "stm32fxxx_hal.h"

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
