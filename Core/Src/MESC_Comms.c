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
#include "MESCfoc.h"
#include <stdio.h>

extern char UART_rx_buffer[2];
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t message_buffer[20];
    uint8_t length;

    __NOP();
    if (UART_rx_buffer[0] == 0x68)
    {  // h - Say hi
        HAL_UART_Transmit_DMA(&huart3, (uint8_t*)"hi", 2);
    }
    else if (UART_rx_buffer[0] == 0x71)
    {  // q - increase quadrature current
        foc_vars.Idq_req[1] = foc_vars.Idq_req[1] + 1.0;
        if ((foc_vars.Idq_req[1] < 1.0) && (foc_vars.Idq_req[1] > -1.0))
        {
            foc_vars.Idq_req[1] = 0;
        }
        length = sprintf((char*) message_buffer, "Ir%.1f %.1f\r", foc_vars.Idq_req[0], foc_vars.Idq_req[1]);
        HAL_UART_Transmit_DMA(&huart3, message_buffer, length);
    }
    else if (UART_rx_buffer[0] == 0x61)
    {  // a - decrease quadrature current
        foc_vars.Idq_req[1] = foc_vars.Idq_req[1] - 1.0;
        if ((foc_vars.Idq_req[1] < 1.0) && (foc_vars.Idq_req[1] > -1.0))
        {
            foc_vars.Idq_req[1] = 0;
        }
        length = sprintf((char*)message_buffer, "Ir%.1f %.1f\r", foc_vars.Idq_req[0], foc_vars.Idq_req[1]);
        HAL_UART_Transmit_DMA(&huart3, message_buffer, length);
    }
    else if (UART_rx_buffer[0] == 0x64)
    {  // d - increase direct (field) current
        foc_vars.Idq_req[0] = foc_vars.Idq_req[0] + 1.0;
        if ((foc_vars.Idq_req[0] < 1.0) && (foc_vars.Idq_req[0] > -1.0))
        {
            foc_vars.Idq_req[0] = 0;
        }
        length = sprintf((char*)message_buffer, "Ir%.1f %.1f\r", foc_vars.Idq_req[0], foc_vars.Idq_req[1]);
        HAL_UART_Transmit_DMA(&huart3, message_buffer, length);
    }
    else if (UART_rx_buffer[0] == 0x63)
    {  // c - decrease direct (field) current
        foc_vars.Idq_req[0] = foc_vars.Idq_req[0] - 1.0;
        if ((foc_vars.Idq_req[0] < 1.0) && (foc_vars.Idq_req[0] > -1.0))
        {
            foc_vars.Idq_req[0] = 0;
        }
        length = sprintf((char*)message_buffer, "Ir%.1f %.1f\r", foc_vars.Idq_req[0], foc_vars.Idq_req[1]);
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
        length = sprintf((char*)message_buffer, "Vbus%.2f\r", measurement_buffers.ConvertedADC[0][1]);
        HAL_UART_Transmit_DMA(&huart3, message_buffer, length);
    }
    else
    {
        HAL_UART_Transmit_DMA(&huart3, "Unrecognised!", 13);
    }

    HAL_UART_Receive_IT(&huart3, UART_rx_buffer, 1);
}
