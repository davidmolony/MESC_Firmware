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

#include "MESCflash.h"

void motor_init()
{
    motor.Rphase = 0;  // We init at 0 to trigger the measurer to get the vals
    motor.Lphase = 0;  // We init at 0 to trigger the measurer to get the vals
    motor.uncertainty = 1;
}

void hw_init()
{
    g_hw_setup.Imax =
        120.0;  // Imax is the current at which we are either no longer able to read it, or hardware "don't ever exceed to avoid breakage"
    g_hw_setup.Vmax = 55.0;  // Headroom beyond which likely to get avalanche of MOSFETs or DCDC converter
    g_hw_setup.Vmin = 10;    // This implies that the PSU has crapped out or a wire has fallen out, and suddenly there will be no power.
    g_hw_setup.Rshunt = 0.0005;
    g_hw_setup.RIphPU = 4700;
    g_hw_setup.RIphSR = 150;
    g_hw_setup.RVBB = 1500;
    g_hw_setup.RVBT = 82000;
    g_hw_setup.OpGain = 16;  // Can this be inferred from the HAL declaration?
    g_hw_setup.VBGain = (3.3f / 4096.0f) * (g_hw_setup.RVBB + g_hw_setup.RVBT) / g_hw_setup.RVBB;
    g_hw_setup.Igain = 3.3 / (g_hw_setup.Rshunt * 4096 * g_hw_setup.OpGain * g_hw_setup.RIphPU / (g_hw_setup.RIphPU + g_hw_setup.RIphSR));
    g_hw_setup.RawCurrLim = g_hw_setup.Imax * g_hw_setup.Rshunt * g_hw_setup.OpGain * (4096 / 3.3) + 2048;
    if (g_hw_setup.RawCurrLim > 4000)
    {
        g_hw_setup.RawCurrLim = 4000;
    }  // 4000 is 96 counts away from ADC saturation, allow headroom for opamp not pulling rail:rail.
    g_hw_setup.RawVoltLim = (uint16_t)(4096.0f * (g_hw_setup.Vmax / 3.3f) * g_hw_setup.RVBB / (g_hw_setup.RVBB + g_hw_setup.RVBT));
    g_hw_setup.battMaxPower = 500.0f;
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
