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
// flash_driver.c
// flash_wrapper.c
// MESCfoc.c
// MESC_Comms.c

#define MESC_GPIO_HALL GPIOB

#define getHallState(...) ((MESC_GPIO_HALL->IDR >> 6) & 0x7)

#endif /* INC_STM32FXXX_HAL_H_ */
