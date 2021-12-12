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

#ifndef MESC_CLI_H
#define MESC_CLI_H

#include <stdint.h>

/*
Command             Description
R <NAME>            Read variable <NAME>
W <NAME> <VALUE>    Write variable <NAME> with <VALUE>
X <NAME>            Execute function <NAME>
I <NAME> <VALUE>    Increment variable <NAME> by <VALUE>
D <NAME> <VALUE>    Decrement variable <NAME> by <VALUE>
F <SIZE> <CHECK>    Flash <SIZE> bytes with checksum <CHECK>
*/

enum CLIVariableType
{
    CLI_VARIABLE_INT,
    CLI_VARIABLE_UINT,
    CLI_VARIABLE_FLOAT,
};

typedef enum CLIVariableType CLIVariableType;

void cli_configure_storage_io(
    int  (* const write )( void const * buffer, uint32_t const address, uint32_t const length )
    );

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

void cli_register_io(
    void * handle,
    int (* const write)( void * handle, void * data, uint16_t size ) ); // NOTE: This prototype is deliberately punned to match HAL_UART_Transmit_DMA

int/*CLIState*/ cli_process( char const c );

void cli_reply( char const * p, ... );

#endif
