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

void motor_init()
{
    motor.Rphase = 0;  // We init at 0 to trigger the measurer to get the vals
    motor.Lphase = 0;  // We init at 0 to trigger the measurer to get the vals
    motor.uncertainty = 1;
}

void hw_init()
{
    g_hw_setup.Imax =
        110.0;  // Imax is the current at which we are either no longer able to read it, or hardware "don't ever exceed to avoid breakage"
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
