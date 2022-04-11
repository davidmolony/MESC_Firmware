/*
 * stm32fxxx_hal.h
 *
 *  Created on: Oct 16, 2021
 *      Author: Lenovo
 */

#ifndef INC_STM32FXXX_HAL_H_
#define INC_STM32FXXX_HAL_H_

#include "stm32f3xx_hal.h"

// TODO move all the 303 specific defines (STM32F303xC) into this from:
// MESCfoc.c
// MESC_Comms.c

/*
Hardware identifiers
*/

#define MESC_GPIO_HALL GPIOB

extern TIM_HandleTypeDef htim17;
#define debugtim htim17

/*
Function prototypes
*/

#define getHallState(...) ((MESC_GPIO_HALL->IDR >> 6) & 0x7)

/*
Profile defaults
*/

/* Temperature parameters */
#define MESC_PROFILE_TEMP_R_F     4700.0f
#define MESC_PROFILE_TEMP_SCHEMA  TEMP_SCHEMA_R_F_ON_R_T
#define MESC_PROFILE_TEMP_SH_BETA 3437.864258f
#define MESC_PROFILE_TEMP_SH_R    0.098243f
#define MESC_PROFILE_TEMP_SH_R0   10000.0f





#endif /* INC_STM32FXXX_HAL_H_ */
