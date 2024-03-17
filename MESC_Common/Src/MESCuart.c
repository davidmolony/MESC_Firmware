/*
* Copyright 2021-2022 cod3b453
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

#include "MESCfoc.h"
#include "MESCmotor_state.h"

#if MESC_UART_USB
#include "usbd_cdc_if.h"
#else
extern UART_HandleTypeDef HW_UART;
#endif


extern TIM_HandleTypeDef  htim1;

extern uint8_t b_read_flash;

#ifndef USE_TTERM

static uint8_t UART_rx_buffer[2];

#if MESC_UART_USB
static void usb_ack( void )
{

}

HAL_StatusTypeDef HAL_USB_Transmit(UART_HandleTypeDef *husb, const uint8_t *pData, uint16_t Size){
	CDC_Transmit_FS((uint8_t*)pData, Size);
	return HAL_OK;
}

#else
static void uart_ack( void )
{
    // Required to allow next receive
    HAL_UART_Receive_IT( &HW_UART, UART_rx_buffer, 1 );
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (UART_rx_buffer[0] == '\r') // Treat CR...
    {
        UART_rx_buffer[0] = '\n'; // ...as LF
    }

    cli_process( UART_rx_buffer[0] );
    /*
    Commands may be executed here
    */
}


void USB_CDC_Callback(uint8_t *buffer, uint32_t len){

	for(int i = 0; i<len; i++){
		if (buffer[i] == '\r') // Treat CR...
		{
			//continue;
			buffer[i] = '\n'; // ...as LF
		}
		cli_process( buffer[i] );
	}

}


static void cmd_hall_dec( void )
{
    for (int i = 0; i < 6; i++)
    {
        mtr[0].m.hall_table[i][2] -= 100;
    }
}

static void cmd_hall_inc( void )
{
    for (int i = 0; i < 6; i++)
    {
    	mtr[0].m.hall_table[i][2] += 100;
    }
}

static void cmd_hello( void )
{
    cli_reply( "%s" "\r" "\n", "HELLO" );
}

static void cmd_stop( void )
{
    cli_reply( "%s" "\r" "\n", "STOP" );
    input_vars.UART_req = 0.0f;
    input_vars.UART_req = 0.0f;
    mtr[0].FOC.Idq_req.d = 0.0f;
    mtr[0].FOC.Idq_req.q = 0.0f;

generateBreak(&mtr[0]);
MotorState = MOTOR_STATE_TRACKING;
}

static void cmd_parameter_setup( void )
{
    MotorState = MOTOR_STATE_DETECTING;

    cli_reply( "Vbus%.2f" "\r" "\n", &mtr[0].Conv.Vbus );
}

static void cmd_reset( void )
{
    cli_reply( "%s" "\r" "\n", "RESET" );

    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY( &htim1 );
    HAL_NVIC_SystemReset();
}

static void cmd_test( void )
{
    MotorState = MOTOR_STATE_TEST;
    phV_Enable(&mtr[0]);
    phW_Enable(&mtr[0]);
}
static void cmd_iqup( void )
{
	input_vars.UART_req = input_vars.UART_req+1.0f;
cli_reply( "%s%f" "\r" "\n", "Iq",mtr[0].FOC.Idq_req.q );
}
static void cmd_iqdown( void )
{
input_vars.UART_req = input_vars.UART_req-1.0f;
cli_reply( "%s%f" "\r" "\n", "Iq",mtr[0].FOC.Idq_req.q );
}
static void cmd_measure( void )
{
    MotorState = MOTOR_STATE_MEASURING;
}
static void cmd_getkV( void )
{
    MotorState = MOTOR_STATE_GET_KV;
}
static void cmd_detect( void )
{
    MotorState = MOTOR_STATE_DETECTING;
}

void uart_init( void )
{
	cli_register_variable_rw( "Idq_req", &input_vars.UART_req, sizeof(input_vars.UART_req), CLI_VARIABLE_FLOAT );
    cli_register_variable_rw( "Id"       , &mtr[0].FOC.Idq_req.d                   , sizeof(mtr[0].FOC.Idq_req.d                   ), CLI_VARIABLE_FLOAT );
    cli_register_variable_rw( "Iq"        , &mtr[0].FOC.Idq_req.q                   , sizeof(mtr[0].FOC.Idq_req.q                   ), CLI_VARIABLE_FLOAT );
    cli_register_variable_ro( "Vbus"      , &mtr[0].Conv.Vbus, sizeof(&mtr[0].Conv.Vbus), CLI_VARIABLE_FLOAT );

    cli_register_function( "hall_dec"     , cmd_hall_dec        );
    cli_register_function( "hall_inc"     , cmd_hall_inc        );
    cli_register_function( "hello"        , cmd_hello           );
    cli_register_function( "stop"         , cmd_stop           	);
    cli_register_function( "param_setup"  , cmd_parameter_setup );
    cli_register_function( "reset"        , cmd_reset           );
    cli_register_function( "test"         , cmd_test            );
    cli_register_function( "q"         	, cmd_iqup            	);
    cli_register_function( "a"         	, cmd_iqdown            );
    cli_register_function( "measure"   	, cmd_measure           );
    cli_register_function( "getkV"   	, cmd_getkV            	);
    cli_register_function( "detect"   , cmd_detect           	);


#if MESC_UART_USB
    cli_register_io( NULL, (int(*)(void *,void *,uint16_t))HAL_USB_Transmit, usb_ack );

#else
    cli_register_io( &HW_UART, (int(*)(void *,void *,uint16_t))HAL_UART_Transmit_DMA, uart_ack );
    // Required to allow initial receive
    uart_ack();
#endif
}

#endif
