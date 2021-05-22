/*
* Copyright 2021 cod3b453
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "MESCuart.h"

#include "MESCcli.h"

#include "MESCmotor_state.h"

extern UART_HandleTypeDef huart3;

extern uint8_t UART_rx_buffer[2];

extern uint8_t b_read_flash;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    cli_process( UART_rx_buffer[0] );
    /*
    Commands may be executed here
    */
    HAL_UART_Receive_IT( &huart3, UART_rx_buffer, 1 );
}

static void cmd_hello( void )
{
    cli_reply( "%s", "HELLO" );
}

static void cmd_parameter_setup( void )
{
    MotorState = MOTOR_STATE_DETECTING;
    b_read_flash = 0;

    cli_reply( "Vbus%.2f", measurement_buffers.ConvertedADC[0][1] );
}

static void cmd_reset( void )
{
    cli_reply( "%s", "RESET" );

    __HAL_TIM_DOE_DISABLE_UNCONDITIONALLY( &htim1 );
    HAL_NVIC_SystemReset();
}

static void cmd_test( void )
{
    MotorState = MOTOR_STATE_TEST;
    phV_Enable();
    phW_Enable();
}

void uart_init( void )
{
    cli_register_variable_rw( "hall_table", &foc_vars.hall_table[i][2]             , sizeof(foc_vars.hall_table[i][2]             ), CLI_VARIABLE_UINT  );
    cli_register_variable_rw( "Idc"       , &foc_vars.Idq_req[0]                   , sizeof(foc_vars.Idq_req[0]                   ), CLI_VARIABLE_FLOAT );
    cli_register_variable_rw( "Iq"        , &foc_vars.Idq_req[1]                   , sizeof(foc_vars.Idq_req[1]                   ), CLI_VARIABLE_FLOAT );
    cli_register_variable_ro( "Vbus"      , &measurement_buffers.ConvertedADC[0][1], sizeof(measurement_buffers.ConvertedADC[0][1]), CLI_VARIABLE_FLOAT );

    cli_register_function( "hello      ", cmd_hello           );
    cli_register_function( "param_setup", cmd_parameter_setup );
    cli_register_function( "reset"      , cmd_reset           );
    cli_register_function( "test"       , cmd_test            );

    cli_register_io( HAL_UART_Transmit_DMA );
}
