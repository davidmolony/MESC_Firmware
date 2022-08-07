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

#include "MESCflash.h"

extern ADC_HandleTypeDef hadc1;

extern TIM_HandleTypeDef htim1;

hw_setup_s g_hw_setup;
motor_s motor;

uint32_t ADC_buffer[6];

void hw_init() {
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
}

void getRawADC(void) {
	//Get the injected critical conversions
  measurement_buffers.RawADC[0][0] = hadc1.Instance->JDR1;  // U Current
  measurement_buffers.RawADC[0][1] = hadc1.Instance->JDR4;  // DC Link Voltage

  measurement_buffers.RawADC[1][0] = hadc1.Instance->JDR2;  // V Current
  measurement_buffers.RawADC[2][0] = hadc1.Instance->JDR3;  // W Current
  
  //These are handled by regular conversion manager and DMA
  measurement_buffers.RawADC[1][3] = ADC_buffer[3];  // Throttle

//Voltage sense for the MP2
  measurement_buffers.RawADC[0][2] = ADC_buffer[0]; //PhaseU Voltage
  measurement_buffers.RawADC[1][1] = ADC_buffer[1]; //PhaseV Voltage
  measurement_buffers.RawADC[1][2] = ADC_buffer[2]; //PhaseW Voltage

//MOS temperature for MP2
  measurement_buffers.RawADC[3][0] = ADC_buffer[5]; //Temperature on PB1
//Motor temp or Brake, needs plumbing in to main MESC...
//   = ADC_buffer[4]
}

void getRawADCVph(void){



}
#if 0
static uint32_t const flash_sector_map[] = {
    // 4 x  16k
    FLASH_BASE + (0 * (16 << 10)),
    FLASH_BASE + (1 * (16 << 10)),
    FLASH_BASE + (2 * (16 << 10)),
    FLASH_BASE + (3 * (16 << 10)),
    // 1 x  64k
    FLASH_BASE + (4 * (16 << 10)),
    // 7 x 128k
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (0 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (1 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (2 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (3 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (4 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (5 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (6 * (128 << 10)),
    // END
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (7 * (128 << 10)),
};

static uint32_t getFlashSectorAddress( uint32_t const index )
{
    return flash_sector_map[index];
}

static uint32_t getFlashSectorIndex(uint32_t const address)
{
    for (uint32_t i = 0; i < ((sizeof(flash_sector_map) / sizeof(*flash_sector_map)) - 1); i++)
    {
        if ((flash_sector_map[i] <= address) && (address < flash_sector_map[i + 1]))
        {
            return i;
        }
    }

    // error
    return UINT32_MAX;
}

uint32_t getFlashBaseAddress( void )
{
/*
The base address is FLASH_BASE = 0x08000000 but this is shared with program
memory and so a suitable offset should be used
*/
    return getFlashSectorAddress( 11 );
}

ProfileStatus eraseFlash( uint32_t const address, uint32_t const length )
{
    // Disallow zero length (could ignore)
    if (length == 0)
    {
        return PROFILE_STATUS_ERROR_DATA_LENGTH;
    }

    uint32_t const saddr = address;
    uint32_t const eaddr = saddr + length - 1;

    uint32_t const ssector = getFlashSectorIndex( saddr );
    uint32_t const esector = getFlashSectorIndex( eaddr );

    // Limit erasure to a single sector
    if (ssector != esector)
    {
        return PROFILE_STATUS_ERROR_DATA_LENGTH;
    }

    FLASH_EraseInitTypeDef sector_erase;

    sector_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    sector_erase.Banks     = FLASH_BANK_1; // (ignored)
    sector_erase.Sector    = ssector;
    sector_erase.NbSectors = (esector - ssector + 1);

    uint32_t bad_sector = 0;

    HAL_StatusTypeDef const sts = HAL_FLASHEx_Erase( &sector_erase, &bad_sector );

    switch (sts)
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
#endif
void mesc_init_1( void )
{
    // Do nothing
}

void mesc_init_2( void )
{
    // Do nothing
}

void mesc_init_3( void )
{
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_buffer, 6);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_ADCEx_InjectedStart(&hadc1);
	__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD);
	__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);
	hadc1.Instance->LTR = 4001; //default for AWD
	hadc1.Instance->LTR = 100;

}
