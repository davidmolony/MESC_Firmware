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

#ifndef MESC_CLI_H
#define MESC_CLI_H

#include <stdint.h>

#include "MESC_INTERNAL.h"
#include "MESC_STM.h"
#include "MESCprofile.h"

/*
Command             Description
R <NAME>            Read variable <NAME>; return value '(dec) 0x(hex)'
W <NAME> <VALUE>    Write variable <NAME> with <VALUE> (dec)
X <NAME>            Execute function <NAME>
I <NAME> <VALUE>    Increment variable <NAME> by <VALUE> (dec)
D <NAME> <VALUE>    Decrement variable <NAME> by <VALUE> (dec)
F <SIZE> <CHECK>    Flash <SIZE> bytes with checksum <CHECK>
P <NAME>            Probe variable <NAME>; return scope index (dec)
*/

enum CLIVariableType
{
    CLI_VARIABLE_INT,
    CLI_VARIABLE_UINT,
    CLI_VARIABLE_FLOAT,
	CLI_VARIABLE_CHAR,
	CLI_VARIABLE_STRING,
	CLI_VARIABLE_BOOL,
};

#define typename(x) _Generic((x), \
    uint8_t:    CLI_VARIABLE_UINT, \
    uint16_t:   CLI_VARIABLE_UINT, \
    uint32_t:   CLI_VARIABLE_UINT, \
    int8_t:     CLI_VARIABLE_INT, \
    int16_t:    CLI_VARIABLE_INT, \
    int32_t:    CLI_VARIABLE_INT, \
	bool:		CLI_VARIABLE_BOOL, \
    float:      CLI_VARIABLE_FLOAT, \
    char:       CLI_VARIABLE_CHAR, \
    char*:      CLI_VARIABLE_STRING)

typedef enum CLIVariableType CLIVariableType;

#ifdef USE_TTERM
typedef void (* cli_callback)(void);


#define cli_register_var_rw(name, var, func) cli_register_variable_rw(name, &var, sizeof(var), typename(var), func)
#define cli_register_var_ro(name, var, func) cli_register_variable_ro(name, &var, sizeof(var), typename(var), func)
#define cli_register_var_wo(name, var, func) cli_register_variable_wo(name, &var, sizeof(var), typename(var), func)


void cli_register_variable_ro(
    char const * name,
    void const * address, uint32_t const size,
    CLIVariableType const type, cli_callback func);

void cli_register_variable_rw(
    char const * name,
    void       * address, uint32_t const size,
    CLIVariableType const type, cli_callback func);

void cli_register_variable_wo(
    char const * name,
    void       * address, uint32_t const size,
    CLIVariableType const type, cli_callback func);

void cli_register_function(
    char const * name,
    void (* const fn)( void), cli_callback func);

#else

#define cli_register_var_rw(name, var) cli_register_variable_rw(name, &var, sizeof(var), typename(var))
#define cli_register_var_ro(name, var) cli_register_variable_ro(name, &var, sizeof(var), typename(var))
#define cli_register_var_wo(name, var) cli_register_variable_wo(name, &var, sizeof(var), typename(var))

void cli_register_variable_ro(
    char const * name,
    void const * address, uint32_t const size,
    CLIVariableType const type );

void cli_register_variable_rw(
    char const * name,
    void       * address, uint32_t const size,
    CLIVariableType const type );

void cli_register_variable_wo(
    char const * name,
    void       * address, uint32_t const size,
    CLIVariableType const type );

void cli_register_function(
    char const * name,
    void (* const fn)( void ) );

#endif

void cli_configure_storage_io(
    ProfileStatus (* const write )( void const * buffer, uint32_t const address, uint32_t const length )
    );


void cli_register_io(
    MESC_STM_ALIAS(void,UART_HandleTypeDef) * handle,
    MESC_STM_ALIAS(int,HAL_StatusTypeDef) (* const write)( MESC_STM_ALIAS(void,UART_HandleTypeDef) * handle, MESC_STM_ALIAS(void,uint8_t) * data, uint16_t size ), // NOTE: This prototype is deliberately punned to match HAL_UART_Transmit_DMA
    void (* const read) ( void ) );

MESC_INTERNAL_ALIAS(int,CLIState) cli_process( char const c );

void cli_reply( char const * p, ... );

void cli_reply_scope( void );


#ifdef USE_TTERM

#include "TTerm/Core/include/TTerm.h"

uint8_t cli_read(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t cli_write(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t cli_list(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
#endif



#endif
