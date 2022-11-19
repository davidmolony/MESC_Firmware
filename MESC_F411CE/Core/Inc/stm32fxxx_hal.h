/*
 * stm32fxxx_hal.h
 *
 *  Created on: Oct 16, 2021
 *      Author: Lenovo
 */

#ifndef INC_STM32FXXX_HAL_H_
#define INC_STM32FXXX_HAL_H_

#include "stm32f4xx_hal.h"
#include "MESC_F401_MP2.h"
// TODO move all the 401 specific defines (STM32F401xx) into this from:
// MESCfoc.c
// MESC_Comms.c

/*
Hardware identifiers
*/
#define SINGLE_ADC
#define MESC_GPIO_HALL GPIOC

extern TIM_HandleTypeDef htim10;
#define debugtim htim10

/*
Function prototypes
*/

#define getHallState(...) ((MESC_GPIO_HALL->IDR >> 13) & 0x7)

/*
Profile defaults
*/

/* Temperature parameters */
#define MESC_PROFILE_TEMP_R_F     10000.0f
#define MESC_PROFILE_TEMP_SCHEMA  TEMP_SCHEMA_R_T_ON_R_F
#define MESC_PROFILE_TEMP_SH_BETA 3437.864258f
#define MESC_PROFILE_TEMP_SH_R    0.098243f
#define MESC_PROFILE_TEMP_SH_R0   10000.0f

#endif /* INC_STM32FXXX_HAL_H_ */
