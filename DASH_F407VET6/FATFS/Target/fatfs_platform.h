/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : fatfs_platform.h
  * @brief          : fatfs_platform header file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* Defines ------------------------------------------------------------------*/
#define SD_PRESENT               ((uint8_t)0x01)  /* also in bsp_driver_sd.h */
#define SD_NOT_PRESENT           ((uint8_t)0x00)  /* also in bsp_driver_sd.h */
#define SD_DETECT_PIN         GPIO_PIN_4
#define SD_DETECT_GPIO_PORT   GPIOE
/* Prototypes ---------------------------------------------------------------*/
uint8_t	BSP_PlatformIsDetected(void);
