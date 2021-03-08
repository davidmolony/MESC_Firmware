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
#include "MESCmotor_state.h"

extern char UART_rx_buffer[2];
extern uint16_t ICVals[2];

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// UART implementation
//////////////////////////////////////////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t message_buffer[20];
    uint8_t length;

    if (UART_rx_buffer[0] == 0x68)
    {  // h - Say hi
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)"hi", 2);
    }
    else if (UART_rx_buffer[0] == 0x71)
    {  // q - increase quadrature current
        foc_vars.Idq_req[1] = foc_vars.Idq_req[1] + 5.0;
        if ((foc_vars.Idq_req[1] < 1.0) && (foc_vars.Idq_req[1] > -1.0))
        {
            foc_vars.Idq_req[1] = 0;
        }
        length = sprintf((char *)message_buffer, "Ir%.1f %.1f\r", foc_vars.Idq_req[0], foc_vars.Idq_req[1]);
        HAL_UART_Transmit_DMA(&huart3, message_buffer, length);
    }
    else if (UART_rx_buffer[0] == 0x61)
    {  // a - decrease quadrature current
        foc_vars.Idq_req[1] = foc_vars.Idq_req[1] - 5.0;
        if ((foc_vars.Idq_req[1] < 1.0) && (foc_vars.Idq_req[1] > -1.0))
        {
            foc_vars.Idq_req[1] = 0;
        }
        length = sprintf((char *)message_buffer, "Ir%.1f %.1f\r", foc_vars.Idq_req[0], foc_vars.Idq_req[1]);
        HAL_UART_Transmit_DMA(&huart3, message_buffer, length);
    }
    else if (UART_rx_buffer[0] == 0x64)
    {  // d - increase direct (field) current
        foc_vars.Idq_req[0] = foc_vars.Idq_req[0] + 5.0;
        if ((foc_vars.Idq_req[0] < 1.0) && (foc_vars.Idq_req[0] > -1.0))
        {
            foc_vars.Idq_req[0] = 0;
        }
        length = sprintf((char *)message_buffer, "Ir%.1f %.1f\r", foc_vars.Idq_req[0], foc_vars.Idq_req[1]);
        HAL_UART_Transmit_DMA(&huart3, message_buffer, length);
    }
    else if (UART_rx_buffer[0] == 0x63)
    {  // c - decrease direct (field) current
        foc_vars.Idq_req[0] = foc_vars.Idq_req[0] - 5.0;
        if ((foc_vars.Idq_req[0] < 1.0) && (foc_vars.Idq_req[0] > -1.0))
        {
            foc_vars.Idq_req[0] = 0;
        }
        length = sprintf((char *)message_buffer, "Ir%.1f %.1f\r", foc_vars.Idq_req[0], foc_vars.Idq_req[1]);
        HAL_UART_Transmit_DMA(&huart3, message_buffer, length);
    }
    else if (UART_rx_buffer[0] == 0x72)
    {  // r - Reset the controller
        HAL_UART_Transmit_DMA(&huart3, "reset!", 6);
        // HAL_Delay(10);
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
        HAL_NVIC_SystemReset();
    }
    else if (UART_rx_buffer[0] == 0x76)
    {  // v - Get the bus voltage
        length = sprintf((char *)message_buffer, "Vbus%.2f\r", measurement_buffers.ConvertedADC[0][1]);
        HAL_UART_Transmit_DMA(&huart3, message_buffer, length);
    }
    else if (UART_rx_buffer[0] == 0x70)
    {  // p - Get the parameters (hall table and L R)
    	MotorState = MOTOR_STATE_DETECTING;
    	extern b_read_flash;
    	b_read_flash = 0;
        length = sprintf((char *)message_buffer, "Vbus%.2f\r", measurement_buffers.ConvertedADC[0][1]);
        HAL_UART_Transmit_DMA(&huart3, message_buffer, length);
    }
    else if (UART_rx_buffer[0] == 0x66)
    {  // f - increase hall table by 100
    	for(int i=0;i<6;i++){
    		foc_vars.hall_table[i][2]=foc_vars.hall_table[i][2]+100;
    	}

    }
    else if (UART_rx_buffer[0] == 0x67)
    {  // g - decrease hall table by 100
    	for(int i=0;i<6;i++){
    		foc_vars.hall_table[i][2]=foc_vars.hall_table[i][2]-100;
    	}
    }
    else
    {
        HAL_UART_Transmit_DMA(&huart3, "Unrecognised!", 13);
    }
    // Restart the UART interrupt, otherwise you're not getting the next byte!
    HAL_UART_Receive_IT(&huart3, UART_rx_buffer, 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// RCPWM implementation
//////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        ICVals[0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

        // Target is 20000 guard is +-10000
        if ((ICVals[0] < 10000) || (30000 < ICVals[0]))
        {
            BLDCVars.ReqCurrent = 0;
            foc_vars.Idq_req[0] = 0;
            foc_vars.Idq_req[1] = 0;
        }

        else if (ICVals[0] != 0)
        {
            BLDCState = BLDC_FORWARDS;
            ICVals[1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
            if (ICVals[1] > 2000) ICVals[1] = 2000;
            if (ICVals[1] < 1000) ICVals[1] = 1000;

            // Mid-point is 1500 guard is +-100
            if ((ICVals[1] > 1400) && (1600 > ICVals[1]))
            {
                ICVals[1] = 1500;
            }
            // Set the current setpoint here
            if (1)
            {  // Current control, ToDo convert to Enum
                if (ICVals[1] > 1600)
                {
                    BLDCVars.ReqCurrent = ((float)ICVals[1] - 1600) / 10.0;
                    foc_vars.Idq_req[0] = 0;
                    foc_vars.Idq_req[1] = ((float)ICVals[1] - 1600) / 5.0;
                }
                // Crude hack, which gets current scaled to +/-80A
                // based on 1000-2000us PWM in
                else if (ICVals[1] < 1400)
                {
                    BLDCVars.ReqCurrent = ((float)ICVals[1] - 1600) / 10.0;
                    foc_vars.Idq_req[0] = 0;
                    foc_vars.Idq_req[1] = ((float)ICVals[1] - 1400) / 10.0;
                }
                // Crude hack, which gets current scaled to +/-40A
                // based on 1000-2000us PWM in
                else
                {
                    static float currentset = 0;
                    BLDCVars.ReqCurrent = 0;
                    foc_vars.Idq_req[0] = 0;
                    foc_vars.Idq_req[1] = currentset;
                }
            }

            /*if (0)
            {  // Duty cycle control, ToDo convert to Enum
                if (a < 10)
                {
                    BLDCVars.BLDCduty = 0;
                }
                if (a > 9)
                {
                    BLDCVars.BLDCduty = 10 * (a - 9);
                }
            }*/
        }
    }
}
