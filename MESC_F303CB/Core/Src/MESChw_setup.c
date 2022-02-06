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

extern ADC_HandleTypeDef hadc1, hadc2, hadc3, hadc4;

#include "MESCflash.h"
#include "MESCfoc.h"

extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

extern TIM_HandleTypeDef htim1;

extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp4;
extern COMP_HandleTypeDef hcomp7;

void motor_init()
{
    motor.Rphase = 0;  // We init at 0 to trigger the measurer to get the vals
    motor.Lphase = 0;  // We init at 0 to trigger the measurer to get the vals
    motor.uncertainty = 1;
}

void hw_init()
{
    g_hw_setup.Imax =
        48.0;  // Imax is the current at which we are either no longer able to read it, or hardware "don't ever exceed to avoid breakage"
    g_hw_setup.Vmax = 50.0;  // Headroom beyond which likely to get avalanche of MOSFETs or DCDC converter
    g_hw_setup.Vmin = 10;    // This implies that the PSU has crapped out or a wire has fallen out, and suddenly there will be no power.
    g_hw_setup.Rshunt = 0.0005;
    g_hw_setup.RIphPU = 4700;
    g_hw_setup.RIphSR = 150;
    g_hw_setup.RVBB = 1500;
    g_hw_setup.RVBT = 82000;
    g_hw_setup.OpGain = 16;  // Can this be inferred from the HAL declaration?
    g_hw_setup.VBGain = (3.3f / 4096.0f) * (g_hw_setup.RVBB + g_hw_setup.RVBT) / g_hw_setup.RVBB;
    g_hw_setup.Igain =
        3.3 / (g_hw_setup.Rshunt * 4096 * g_hw_setup.OpGain * SHUNT_POLARITY * g_hw_setup.RIphPU / (g_hw_setup.RIphPU + g_hw_setup.RIphSR));
    g_hw_setup.RawCurrLim = g_hw_setup.Imax * g_hw_setup.Rshunt * g_hw_setup.OpGain * (4096 / 3.3) + 2048;
    if (g_hw_setup.RawCurrLim > 4000)
    {
        g_hw_setup.RawCurrLim = 4000;
    }  // 4000 is 96 counts away from ADC saturation, allow headroom for opamp not pulling rail:rail.
    g_hw_setup.RawVoltLim = (uint16_t)(4096.0f * (g_hw_setup.Vmax / 3.3f) * g_hw_setup.RVBB / (g_hw_setup.RVBB + g_hw_setup.RVBT));
    g_hw_setup.battMaxPower = 500.0f;
}

void setAWDVals()
{
    uint32_t AWD_top_set_point = g_hw_setup.RawCurrLim - 2048 + measurement_buffers.ADCOffset[0];
    if (AWD_top_set_point > 4000)
    {
        AWD_top_set_point = 4000;
    }

    uint32_t AWD_bottom_set_point = measurement_buffers.ADCOffset[0] - (g_hw_setup.RawCurrLim - 2048);
    if (AWD_bottom_set_point<96 | AWD_bottom_set_point> measurement_buffers.ADCOffset[0])
    {
        AWD_bottom_set_point = 96;
    }

    uint32_t AWD_setpoints = 0;
    AWD_setpoints |= AWD_bottom_set_point;
    AWD_setpoints |= (AWD_top_set_point << 16);

    hadc1.Instance->TR1 = AWD_setpoints;
    AWD_top_set_point = g_hw_setup.RawCurrLim - 2048 + measurement_buffers.ADCOffset[1];
    if (AWD_top_set_point > 4000)
    {
        AWD_top_set_point = 4000;
    }

    AWD_bottom_set_point = measurement_buffers.ADCOffset[1] - (g_hw_setup.RawCurrLim - 2048);
    if (AWD_bottom_set_point<96 | AWD_bottom_set_point> measurement_buffers.ADCOffset[1])
    {
        AWD_bottom_set_point = 96;
    }

    AWD_setpoints = 0;
    AWD_setpoints |= AWD_bottom_set_point;
    AWD_setpoints |= (AWD_top_set_point << 16);
    hadc2.Instance->TR1 = AWD_setpoints;

    AWD_top_set_point = g_hw_setup.RawCurrLim - 2048 + measurement_buffers.ADCOffset[2];
    if (AWD_top_set_point > 4000)
    {
        AWD_top_set_point = 4000;
    }

    AWD_bottom_set_point = measurement_buffers.ADCOffset[2] - (g_hw_setup.RawCurrLim - 2048);
    if (AWD_bottom_set_point<96 | AWD_bottom_set_point> measurement_buffers.ADCOffset[2])
    {
        AWD_bottom_set_point = 96;
    }

    AWD_setpoints = 0;
    AWD_setpoints |= AWD_bottom_set_point;
    AWD_setpoints |= (AWD_top_set_point << 16);
    hadc3.Instance->TR1 = AWD_setpoints;
    // Over Voltage
    hadc1.Instance->TR2 = (g_hw_setup.RawVoltLim << 12);  // AWD2 is an oddity since it only uses the top 8 bits
    hadc1.Instance->AWD2CR = 0x2;                         // Set channel 0 as the guarded channel
}

void getRawADC(void)
{
    // Do not need to do anything here; handled by DMA
}

uint32_t getFlashBaseAddress( void )
{
/*
The base address is FLASH_BASE = 0x08000000 but this is shared with program
memory and so the final page is used for profile storage
(see STM32F303CBTX_FLASH.ld where this region is mapped in NVM and FLASH is
reduced accordingly)
*/
    return 0x0801F800;
}

static uint32_t getFlashPageAddress( uint32_t const index )
{
    return (getFlashBaseAddress() + (index * FLASH_PAGE_SIZE));
}

static uint32_t getFlashPageIndex( uint32_t const address )
{
    return (address / FLASH_PAGE_SIZE);
}

ProfileStatus eraseFlash( uint32_t const address, uint32_t const length )
{
    // Disallow zero length (could ignore)
    if (length == 0)
    {
        return PROFILE_STATUS_ERROR_DATA_LENGTH;
    }

    uint32_t const spage = getFlashPageIndex( address              );
    uint32_t const epage = getFlashPageIndex( address + length - 1 );

    // Limit erasure to a single page
    if (spage != epage)
    {
        return PROFILE_STATUS_ERROR_DATA_LENGTH;
    }

    FLASH_EraseInitTypeDef page_erase;

    page_erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    page_erase.PageAddress = getFlashPageAddress( spage );
    page_erase.NbPages     = (epage - spage + 1);

    uint32_t result = 0;

    HAL_FLASHEx_Erase( &page_erase, &result );

    switch (result)
    {
        case HAL_OK:
            return PROFILE_STATUS_SUCCESS;
        case HAL_ERROR:
            return PROFILE_STATUS_ERROR_STORAGE_WRITE;
        case HAL_BUSY:
        case HAL_TIMEOUT:
        default:
            return PROFILE_STATUS_UNKNOWN;
    }
}

void mesc_init_1( void )
{
    HAL_ADCEx_Calibration_Start( &hadc1, ADC_SINGLE_ENDED );
    HAL_ADCEx_Calibration_Start( &hadc2, ADC_SINGLE_ENDED );
    HAL_ADCEx_Calibration_Start( &hadc3, ADC_SINGLE_ENDED );
    HAL_ADCEx_Calibration_Start( &hadc4, ADC_SINGLE_ENDED );
}

void mesc_init_2( void )
{
    HAL_OPAMP_Start( &hopamp1 );
    HAL_OPAMP_Start( &hopamp2 );
    HAL_OPAMP_Start( &hopamp3 );
}

void mesc_init_3( void )
{
    HAL_TIM_PWM_Start(     &htim1, TIM_CHANNEL_1 );
    HAL_TIMEx_PWMN_Start(  &htim1, TIM_CHANNEL_1 );
    __HAL_TIM_SET_COUNTER( &htim1, 10);

    HAL_TIM_PWM_Start(     &htim1, TIM_CHANNEL_2 );
    HAL_TIMEx_PWMN_Start(  &htim1, TIM_CHANNEL_2 );
    __HAL_TIM_SET_COUNTER( &htim1, 10);

    HAL_TIM_PWM_Start(     &htim1, TIM_CHANNEL_3 );
    HAL_TIMEx_PWMN_Start(  &htim1, TIM_CHANNEL_3 );
    __HAL_TIM_SET_COUNTER( &htim1, 10);

    __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_4, 1022 );

    // Initialise the comparators - 3 overcurrent and 1 overvoltage,
    HAL_COMP_Start( &hcomp1 );
    HAL_COMP_Start( &hcomp2 );
    HAL_COMP_Start( &hcomp4 );
    // HAL_COMP_Start(&hcomp7);  // OVP comparator, may be unwanted if operating
    // above the divider threshold, the ADC conversion can also be used to trigger
    // a protection event

    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&measurement_buffers.RawADC[0][0], 3);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&measurement_buffers.RawADC[1][0], 4);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&measurement_buffers.RawADC[2][0], 1);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_ADC_Start_DMA(&hadc4, (uint32_t *)&measurement_buffers.RawADC[3][0], 1);

    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD1);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD2);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_AWD1);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_AWD1);
    // Using the ADC AWD to detect overcurrent events.

    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    // Using the timer updates to commute the motor
}
