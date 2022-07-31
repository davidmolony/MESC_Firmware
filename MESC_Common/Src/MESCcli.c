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

#define MESC_STM_FIXUP
#include "MESC_STM.h"

#include "MESCcli.h"
#include "MESCfnv.h"
#include "MESCprofile.h"

#include "bit_op.h"
#include "pp_op.h"

#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAKE_TYPE_SIZE(type,size)      ((uint32_t)((uint32_t)((type) << BITS_PER_NYBBLE) | ((uint32_t)(size))))
#define MAKE_TYPE_SIZE_CASE(type,size) ((uint32_t)((uint32_t)((JOIN( CLI_VARIABLE_, type)) << BITS_PER_NYBBLE) | ((uint32_t)(size))))

enum CLIState
{
    CLI_STATE_IDLE,

    CLI_STATE_ABORT,

    CLI_STATE_COMMAND,
    CLI_STATE_VARIABLE,
    CLI_STATE_VALUE,

    CLI_STATE_PARAM_1,
    CLI_STATE_PARAM_2,

    CLI_STATE_EXECUTE,

    CLI_STATE_DATA,
};

typedef enum CLIState CLIState;

static CLIState cli_state;

enum CLICommand
{
    CLI_COMMAND_DECREASE = 'D',
    CLI_COMMAND_FLASH    = 'F',
    CLI_COMMAND_INCREASE = 'I',
    CLI_COMMAND_PROBE    = 'P',
    CLI_COMMAND_READ     = 'R',
    CLI_COMMAND_WRITE    = 'W',
    CLI_COMMAND_EXECUTE  = 'X',
};

typedef enum CLICommand CLICommand;

static CLICommand cli_cmd;

static bool     cli_hash_valid = false;
static uint32_t cli_hash = 0;

// Flash functions
static ProfileStatus cli_flash_write_noop( void const * buffer, uint32_t const address, uint32_t const length )
{
    (void)buffer;
    (void)address;
    (void)length;
    return PROFILE_STATUS_UNKNOWN;
}

static ProfileStatus (* cli_flash_write)( void const * buffer, uint32_t const address, uint32_t const length ) = &cli_flash_write_noop;

// Expected values
static uint32_t cli_flash_exp_len = 0;
static uint32_t cli_flash_exp_chk = 0;
// Current values
static uint32_t cli_flash_cur_off = 0;
static uint32_t cli_flash_cur_chk = 0;

static uint8_t cli_flash_buffer[PROFILE_MAX_SIZE];

enum CLIAccess
{
    CLI_ACCESS_NONE = 0x0,

    CLI_ACCESS_R    = 0x1,
    CLI_ACCESS_W    = 0x2,
    CLI_ACCESS_X    = 0x4,

    CLI_ACCESS_RO   = CLI_ACCESS_R,
    CLI_ACCESS_WO   = CLI_ACCESS_W,
    CLI_ACCESS_RW   = (CLI_ACCESS_R | CLI_ACCESS_W),

    CLI_ACCESS_PROBE = 0x8,
};

typedef enum CLIAccess CLIAccess;

enum CLIVarState
{
    CLI_VAR_STATE_IDLE       = 0,
    // int
    CLI_VAR_STATE_INT        = 1,
    // uint
    CLI_VAR_STATE_UINT       = 1,
    // hex
    // ...
    // float
    CLI_VAR_STATE_FLOAT_SIGN = 1,
    CLI_VAR_STATE_FLOAT_INT  = 2,
    CLI_VAR_STATE_FLOAT_FRAC = 3,
    // ...
};

typedef enum CLIVarState CLIVarState;

struct CLIVar
{
    CLIVarState     state;
    union
    {
    int32_t         i;
    uint32_t        u;
    float           f;
    }               var;
};

typedef struct CLIVar CLIVar;

static CLIVar cli_var = {0};

struct CLIEntry
{
    uint32_t        hash;
    union
    {
    void       *    w;
    void const *    r;
    void (*         x)( void );
    }               var;
    uint32_t        size;
    CLIAccess       access;
    CLIVariableType type;
};

typedef struct CLIEntry CLIEntry;

#define MAX_CLI_LUT_ENTRIES UINT32_C(32)

static CLIEntry   cli_lut[MAX_CLI_LUT_ENTRIES];
static uint32_t   cli_lut_entries = 0;
static CLIEntry * cli_lut_entry = NULL;

static MESC_STM_ALIAS(int,UART_HandleTypeDef) cli_io_write_noop( MESC_STM_ALIAS(void,UART_HandleTypeDef) * handle, MESC_STM_ALIAS(void,uint8_t) * data, uint16_t size )
{
    (void)handle;
    (void)data;
    (void)size;

    return HAL_OK;
}

static void cli_io_read_noop( void )
{

}

static MESC_STM_ALIAS(void,UART_HandleTypeDef) *  cli_io_handle = NULL;
static MESC_STM_ALIAS(int ,HAL_StatusTypeDef ) (* cli_io_write)( MESC_STM_ALIAS(void,UART_HandleTypeDef) *, MESC_STM_ALIAS(void,uint8_t) *, uint16_t ) = cli_io_write_noop;
static void                                    (* cli_io_read )( void ) = cli_io_read_noop;

static void cli_idle( void )
{
    cli_state = CLI_STATE_IDLE;
    cli_hash_valid = false;
    cli_hash = 0;
    cli_var.state = CLI_VAR_STATE_IDLE;
    cli_var.var.u = 0;
}

static void cli_abort( void )
{
    cli_state = CLI_STATE_ABORT;
    cli_hash_valid = false;
    cli_hash = 0;
}

static void cli_noop_write( char const c )
{
    cli_abort();
    (void)c;
}

static void (* cli_process_write_value)( char const ) = cli_noop_write;

static void cli_noop_read( void )
{
    cli_abort();
}

static void (* cli_process_read_value)( void ) = cli_noop_read;

static void cli_process_write_float( char const c ); // Forward declare

static void cli_execute( void )
{
    switch (cli_cmd)
    {
        case CLI_COMMAND_READ:
            cli_process_read_value();
            break;
        case CLI_COMMAND_WRITE:
            if (cli_lut_entry->type == CLI_VARIABLE_FLOAT)
            {
                cli_process_write_float( '\0' );
            }
            memcpy( cli_lut_entry->var.w, &cli_var.var, cli_lut_entry->size );
            break;
        case CLI_COMMAND_EXECUTE:
            cli_lut_entry->var.x();
            break;
        case CLI_COMMAND_INCREASE:
        case CLI_COMMAND_DECREASE:
        {
            CLIVariableType const type = cli_lut_entry->type;
            uint32_t        const size = cli_lut_entry->size;
            union
            {
                int8_t   i8;
                uint8_t  u8;

                int16_t  i16;
                uint16_t u16;

                int32_t  i32;
                uint32_t u32;

                float    f32;
            } * tmp = cli_lut_entry->var.w;

            switch (MAKE_TYPE_SIZE( type, size ))
            {
                case MAKE_TYPE_SIZE_CASE(  INT, sizeof(int8_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->i8 += (int8_t)cli_var.var.i;
                    }
                    else
                    {
                        tmp->i8 -= (int8_t)cli_var.var.i;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE(  INT, sizeof(int16_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->i16 += (int16_t)cli_var.var.i;
                    }
                    else
                    {
                        tmp->i16 -= (int16_t)cli_var.var.i;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE(  INT, sizeof(int32_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->i32 += (int32_t)cli_var.var.i;
                    }
                    else
                    {
                        tmp->i32 -= (int32_t)cli_var.var.i;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( UINT, sizeof(uint8_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->u8 += (uint8_t)cli_var.var.u;
                    }
                    else
                    {
                        tmp->u8 -= (uint8_t)cli_var.var.u;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( UINT, sizeof(uint16_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->u16 += (uint16_t)cli_var.var.u;
                    }
                    else
                    {
                        tmp->u16 -= (uint16_t)cli_var.var.u;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( UINT, sizeof(uint32_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->u32 += (uint32_t)cli_var.var.u;
                    }
                    else
                    {
                        tmp->u32 -= (uint32_t)cli_var.var.u;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( FLOAT, sizeof(float) ):
                    cli_process_write_float( '\0' );
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->f32 += (float)cli_var.var.f;
                    }
                    else
                    {
                        tmp->f32 -= (float)cli_var.var.f;
                    }
                    break;
                default:
                    return;
            }

            break;
        }
        case CLI_COMMAND_FLASH:
            break;
        default:
            // error
            break;
    }

    cli_idle();
}

static CLIEntry * cli_lut_alloc( char const * name )
{
    if (cli_lut_entries >= MAX_CLI_LUT_ENTRIES)
    {
        return NULL;
    }

    uint32_t const hash = fnv1a_str( name );

    for ( uint32_t i = 0; i < cli_lut_entries; ++i)
    {
        if (cli_lut[i].hash == hash)
        {
            // ERROR
            return NULL;
        }
    }

    CLIEntry * entry = &cli_lut[cli_lut_entries];
    entry->hash = hash;

    return entry;
}

static void cli_process_read_xint( void )
{
    char cli_buffer[32];

    if (cli_lut_entry != NULL)
    {
        CLIVariableType const type = cli_lut_entry->type;
        uint32_t        const size = cli_lut_entry->size;
        char * fmt_10 = NULL;
        char * fmt_16 = NULL;

        switch (MAKE_TYPE_SIZE( type, size ))
        {
            case MAKE_TYPE_SIZE_CASE(  INT, 1 ):
                fmt_10 = PRId8;
                fmt_16 = PRIX8;
                break;
            case MAKE_TYPE_SIZE_CASE(  INT, 2 ):
                fmt_10 = PRId16;
                fmt_16 = PRIX16;
                break;
            case MAKE_TYPE_SIZE_CASE(  INT, 4 ):
                fmt_10 = PRId32;
                fmt_16 = PRIX32;
                break;
            case MAKE_TYPE_SIZE_CASE( UINT, 1 ):
                fmt_10 = PRIu8;
                fmt_16 = PRIX8;
                break;
            case MAKE_TYPE_SIZE_CASE( UINT, 2 ):
                fmt_10 = PRIu16;
                fmt_16 = PRIX16;
                break;
            case MAKE_TYPE_SIZE_CASE( UINT, 4 ):
                fmt_10 = PRIu32;
                fmt_16 = PRIX32;
                break;
            default:
                return;
        }

        uint32_t const nybbles = NYBBLES_PER_BYTE * size;
        uint32_t v = UINT32_C(0);

        memcpy( &v, cli_lut_entry->var.r, size );

        if (cli_cmd == CLI_COMMAND_PROBE)
        {
            // %0<nybbles><fmt_16>
            sprintf( cli_buffer,
                "%%" "0%" PRIu32 "%s",
                nybbles, fmt_16 );
            // %0nPRIxXX
            cli_reply( cli_buffer, v );
        }
        else
        {
            // %<fmt_10> 0x%0<nybbles><fmt_16>
            sprintf( cli_buffer,
                "%%" "%s"
                " 0x" "%%" "0%" PRIu32 "%s"
                "\r" "\n",
                fmt_10,
                nybbles, fmt_16 );
            // %PRIxXX 0x%0nPRIxXX
            cli_reply( cli_buffer, v, v );
        }
    }
}

static void cli_process_write_int( char const c )
{
    if ((cli_var.state == CLI_VAR_STATE_IDLE) && (c == '-'))
    {
        cli_var.var.i = INT32_C(-1);
        cli_var.state = CLI_VAR_STATE_INT;
    }

    if (('0' <= c) && (c <= '9'))
    {
        if (cli_var.state == CLI_VAR_STATE_IDLE)
        {
            cli_var.var.i = INT32_C(0);
        }
        cli_var.var.i *= INT32_C(10);
        cli_var.var.i += (c - '0');
        cli_var.state = CLI_VAR_STATE_INT;
    }
    else if (cli_var.state == CLI_VAR_STATE_IDLE)
    {
        cli_abort();
    }
}

static void cli_process_write_uint( char const c )
{
    if (('0' <= c) && (c <= '9'))
    {
        if (cli_var.state == CLI_VAR_STATE_IDLE)
        {
            cli_var.var.u = UINT32_C(0);
        }
        cli_var.var.u *= UINT32_C(10);
        cli_var.var.u += (c - '0');
        cli_var.state = CLI_VAR_STATE_UINT;
    }
    else
    {
        cli_abort();
    }
}

static void cli_process_write_hex( char const c )
{
    if (cli_var.state < (NYBBLES_PER_BYTE * sizeof(cli_var.var.u)))
    {
        cli_var.state++;

        cli_var.var.u = (cli_var.var.u << BITS_PER_NYBBLE);
        
        if (('0' <= c) && (c <= '9'))
        {
            cli_var.var.u = (cli_var.var.u | ((c - '0'     ) & BIT_MASK_32(BITS_PER_NYBBLE)));
        }
        else if (('A' <= c) && (c <= 'F'))
        {
            cli_var.var.u = (cli_var.var.u | ((c - 'A' + 10) & BIT_MASK_32(BITS_PER_NYBBLE)));
        }
        else if (('a' <= c) && (c <= 'f'))
        {
            cli_var.var.u = (cli_var.var.u | ((c - 'a' + 10) & BIT_MASK_32(BITS_PER_NYBBLE)));
        }
        else
        {
            cli_abort();
        }
    }
    else
    {
        cli_abort();
    }
}

static void cli_process_read_float( void )
{
    if (cli_lut_entry != NULL)
    {
        CLIVariableType const type = cli_lut_entry->type;
        uint32_t        const size = cli_lut_entry->size;

        switch (MAKE_TYPE_SIZE( type, size ))
        {
            case MAKE_TYPE_SIZE_CASE( FLOAT, 4 ):
                break;
            default:
                return;
        }

        cli_reply( "%f" "\r" "\n", *((float const *)cli_lut_entry->var.r) );
    }
}

static void cli_process_write_float( char const c )
{
    static float sgn = 0.0f;

    if (c == '\0')
    {
        cli_var.var.f *= sgn;
        sgn = 0.0f;
        return;
    }

    switch (cli_var.state)
    {
        case CLI_VAR_STATE_IDLE:
            cli_var.state = CLI_VAR_STATE_FLOAT_SIGN;
            switch (c)
            {
                case '-':
                    sgn = -1.0f;
                    break;
                default:
                    if (('0' <= c) && (c <= '9'))
                    {
                        cli_var.var.f = ((float)(c - '0'));
                        cli_var.state = CLI_VAR_STATE_FLOAT_INT;
                    }
                    else
                    {
                        cli_abort();
                    }
                    // fallthrough
                case '+':
                    sgn = +1.0f;
                    break;
            }
            break;
        case CLI_VAR_STATE_FLOAT_SIGN:
            cli_var.state = CLI_VAR_STATE_FLOAT_INT;
            if (('0' <= c) && (c <= '9'))
            {
                cli_var.var.f = ((float)(c - '0'));
            }
            else
            {
                cli_abort();
            }
            break;
        case CLI_VAR_STATE_FLOAT_INT:
            if (('0' <= c) && (c <= '9'))
            {
                cli_var.var.f *= 10.0f;
                cli_var.var.f += ((float)(c - '0'));
            }
            else if (c == '.')
            {
                cli_var.state = CLI_VAR_STATE_FLOAT_FRAC;
            }
            else
            {
                cli_abort();
            }
            break;
        case CLI_VAR_STATE_FLOAT_FRAC:
        default:
            if (('0' <= c) && (c <= '9'))
            {
                cli_var.var.f += ((float)(c - '0')) / powf( 10.0f, (float)(cli_var.state - CLI_VAR_STATE_FLOAT_INT)  );
                cli_var.state++;
            }
            else
            {
                cli_abort();
            }
            break;
    }
}

void cli_configure_storage_io(
    ProfileStatus (* const write)( void const * buffer, uint32_t const address, uint32_t const length )
    )
{
    if (write != NULL)
    {
        cli_flash_write = write;
    }
}

void cli_register_variable_ro(
    char const * name,
    void const * address, uint32_t const size,
    CLIVariableType const type )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->var.r  = address;
        entry->size   = size;
        entry->access = CLI_ACCESS_RO;
        entry->type   = type;

        cli_lut_entries++;
    }
}

void cli_register_variable_rw(
    char const * name,
    void       * address, uint32_t const size,
    CLIVariableType const type )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->var.w  = address;
        entry->size   = size;
        entry->access = CLI_ACCESS_RW;
        entry->type   = type;

        cli_lut_entries++;
    }
}

void cli_register_variable_wo(
    char const * name,
    void       * address, uint32_t const size,
    CLIVariableType const type )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->var.w  = address;
        entry->size   = size;
        entry->access = CLI_ACCESS_WO;
        entry->type   = type;

        cli_lut_entries++;
    }
}

void cli_register_function(
    char const * name,
    void (* const fn)( void ) )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->var.x  = fn;
        entry->size   = 0;
        entry->access = CLI_ACCESS_X;
        entry->type   = (CLIVariableType)INT_MAX;

        cli_lut_entries++;
    }
}

void cli_register_io(
    MESC_STM_ALIAS(void,UART_HandleTypeDef) * handle,
    MESC_STM_ALIAS(int,HAL_StatusTypeDef) (* const write)( MESC_STM_ALIAS(void,UART_HandleTypeDef) * handle, MESC_STM_ALIAS(void,uint8_t) * data, uint16_t size ),
    void (* const read)( void ) )
{
    cli_io_handle = handle;
    cli_io_write  = write;
    cli_io_read   = read;
}

static uint32_t cli_lookup_index;

static CLIEntry * cli_lookup( uint32_t const hash )
{
    for ( cli_lookup_index = 0; cli_lookup_index < cli_lut_entries; ++cli_lookup_index )
    {
        if (cli_lut[cli_lookup_index].hash == hash)
        {
            return &cli_lut[cli_lookup_index];
        }
    }

    return NULL;
}

static void (* cli_process_write_type( CLIVariableType const type))( char const c )
{
    switch (type)
    {
        case CLI_VARIABLE_INT:
            return cli_process_write_int;
        case CLI_VARIABLE_UINT:
            return cli_process_write_uint;
        case CLI_VARIABLE_FLOAT:
            return cli_process_write_float;
        default:
            return cli_noop_write;
    }
}

static void (* cli_process_read_type( CLIVariableType const type))( void )
{
    switch (type)
    {
        case CLI_VARIABLE_INT:
        case CLI_VARIABLE_UINT:
            return cli_process_read_xint;
        case CLI_VARIABLE_FLOAT:
            return cli_process_read_float;
        default:
            return cli_noop_read;
    }
}

static void cli_process_variable( const char c )
{
    switch (c)
    {
        case '\n':
        case ' ':
        {
            CLIAccess access = CLI_ACCESS_NONE;

            cli_process_write_value = cli_noop_write;
            cli_process_read_value  = cli_noop_read;

            switch (cli_cmd)
            {
                case CLI_COMMAND_READ:
                case CLI_COMMAND_PROBE:
                    access = CLI_ACCESS_R;
                    break;
                case CLI_COMMAND_WRITE:
                    access = CLI_ACCESS_W;
                    break;
                case CLI_COMMAND_EXECUTE:
                    access = CLI_ACCESS_X;
                    break;
                case CLI_COMMAND_INCREASE:
                case CLI_COMMAND_DECREASE:
                    access = CLI_ACCESS_RW;
                    break;
                default:
                    break;
            }

            cli_lut_entry = cli_lookup( cli_hash );

            if  (
                    (cli_lut_entry == NULL)
                ||  ((cli_lut_entry->access & access) != access)
                )
            {
                cli_abort();
                break;
            }

            switch (cli_cmd)
            {
                case CLI_COMMAND_READ:
                    cli_process_read_value = cli_process_read_type( cli_lut_entry->type );
                    // fallthrough
                case CLI_COMMAND_EXECUTE:
                    cli_state = CLI_STATE_EXECUTE;

                    if  (c == '\n')
                    {
                        if (cli_hash_valid)
                        {
                            cli_execute();
                        }
                        else
                        {
                            cli_abort();
                        }
                    }

                    break;
                case CLI_COMMAND_WRITE:
                    cli_process_write_value = cli_process_write_type( cli_lut_entry->type );

                    cli_state = CLI_STATE_VALUE;
                    cli_hash_valid = false;
                    cli_hash = 0;

                    break;
                case CLI_COMMAND_INCREASE:
                case CLI_COMMAND_DECREASE:
                    cli_process_write_value = cli_process_write_type( cli_lut_entry->type );
                    cli_process_read_value  = cli_process_read_type(  cli_lut_entry->type );

                    cli_state = CLI_STATE_VALUE;
                    cli_hash_valid = false;
                    cli_hash = 0;

                    break;
                case CLI_COMMAND_PROBE:
                    cli_lut_entry->access |= CLI_ACCESS_PROBE;
                    cli_reply( "%" PRIu32 , cli_lookup_index );
                    break;
                default:
                    cli_abort();
                    break;
            }
            break;
        }
        default:
        {
            if (cli_hash_valid)
            {
                if  (
                        (('0' <= c ) && (c <= '9'))
                    ||  (('A' <= c ) && (c <= 'Z'))
                    ||  (('a' <= c ) && (c <= 'z'))
                    ||  (c == '_')
                    )
                {
                    cli_hash = fnv1a_process( cli_hash, c );
                }
                else
                {
                    cli_idle();
                }
            }
            else
            {
                if  (
                        (('A' <= c ) && (c <= 'Z'))
                    ||  (('a' <= c ) && (c <= 'z'))
                    )
                {
                    cli_hash = fnv1a_init();
                    cli_hash = fnv1a_process( cli_hash, c );
                    cli_hash_valid = true;
                }
                else if (c == '\n')
                {
                    cli_idle();
                }
                else
                {
                    cli_abort();
                }
            }
            break;
        }
    }
}

static uint32_t byte_swap(uint32_t const value)
{
    return  (
                ((value & UINT32_C(0x000000FF)) << 24)
            |   ((value & UINT32_C(0x0000FF00)) <<  8)
            |   ((value & UINT32_C(0x00FF0000)) >>  8)
            |   ((value & UINT32_C(0xFF000000)) >> 24)
        )   ;
}

static uint8_t  cli_reply_buffer[256] = {0};
static uint16_t cli_reply_buffer_offset = UINT16_C(0);

static void cli_reply_begin( void )
{
    cli_reply_buffer[0] = '\0';
    cli_reply_buffer_offset = 0;
}

static void cli_reply_close( void )
{
    if (cli_reply_buffer_offset > 0)
    {
        int const ret = cli_io_write( cli_io_handle, cli_reply_buffer, cli_reply_buffer_offset );
        (void)ret;
    }
}

MESC_INTERNAL_ALIAS(int,CLIState) cli_process( char const c )
{
    cli_reply_begin();

    if (c == '\n')
    {
        cli_reply( "%s", "\r\n" );
    }
    else if (cli_state != CLI_STATE_DATA)
    {
        cli_reply( "%c", c );
    }

    switch (cli_state)
    {
        case CLI_STATE_IDLE:
            switch (c)
            {
                case '\n':
                    break;
                case CLI_COMMAND_READ:
                case CLI_COMMAND_WRITE:
                case CLI_COMMAND_EXECUTE:
                case CLI_COMMAND_INCREASE:
                case CLI_COMMAND_DECREASE:
                case CLI_COMMAND_FLASH:
                case CLI_COMMAND_PROBE:
                    cli_cmd = (CLICommand)c;
                    cli_state = CLI_STATE_COMMAND;
                    break;
                default:
                    cli_abort();
                    break;
            }
            break;
        case CLI_STATE_ABORT:
            if (c == '\n')
            {
                cli_idle();
            }
            break;
        case CLI_STATE_COMMAND:
            switch (c)
            {
                case '\n':
                    cli_idle();
                    break;
                case ' ':
                    if (cli_cmd == CLI_COMMAND_FLASH)
                    {
                        cli_flash_exp_len = 0;
                        cli_flash_exp_chk = 0;

                        cli_flash_cur_off = 0;
                        cli_flash_cur_chk = fnv1a_init();

                        cli_state = CLI_STATE_PARAM_1;
                    }
                    else
                    {
                        cli_state = CLI_STATE_VARIABLE;
                    }
                    break;
                default:
                    cli_abort();
                    break;
            }
            break;
        case CLI_STATE_VARIABLE:
            cli_process_variable( c );
            break;
        case CLI_STATE_VALUE:
            if (c == ' ')
            {
                cli_state = CLI_STATE_EXECUTE;
            }
            else if (c != '\n')
            {
                cli_process_write_value( c );
                break;
            }
            // fallthrough
        case CLI_STATE_EXECUTE:
            switch (c)
            {
                case '\n':
                    cli_execute();
                    break;
                case ' ':
                    break;
                default:
                    cli_abort();
                    break;
            }
            break;
        case CLI_STATE_PARAM_1:
            if (c == ' ')
            {
                cli_flash_exp_len = cli_var.var.u;

                cli_var.state = CLI_VAR_STATE_IDLE;
                cli_var.var.u = 0;

                cli_state = CLI_STATE_PARAM_2;
            }
            else
            {
                cli_process_write_hex( c );
            }
            break;
        case CLI_STATE_PARAM_2:
            if (c == '\n')
            {
                cli_flash_exp_chk = cli_var.var.u;

                cli_var.state = CLI_VAR_STATE_IDLE;
                cli_var.var.u = 0;

                if ((cli_flash_exp_len % sizeof(cli_var.var.u)) == 0)
                {
                    cli_state = CLI_STATE_DATA;
                }
                else
                {
                    cli_abort();
                }
            }
            else
            {
                cli_process_write_hex( c );
            }
            break;
        case CLI_STATE_DATA:
            cli_process_write_hex( c );

            if (cli_var.state == (NYBBLES_PER_BYTE * sizeof(cli_var.var.u)))
            {
                cli_var.var.u = byte_swap( cli_var.var.u );

                cli_flash_cur_chk = fnv1a_process_data( cli_flash_cur_chk, &cli_var.var.u, sizeof(cli_var.var.u) );

                *((uint32_t *)(&cli_flash_buffer[cli_flash_cur_off])) = cli_var.var.u;
                cli_flash_cur_off = cli_flash_cur_off + sizeof(cli_var.var.u);

                cli_var.state = CLI_VAR_STATE_IDLE;
                cli_var.var.u = 0;
            }

            if  (
                    (cli_flash_cur_off > cli_flash_exp_len)
                ||  (
                        (cli_flash_cur_off == cli_flash_exp_len)
                    &&  (cli_flash_cur_chk != cli_flash_exp_chk)
                    )
                )
            {
                cli_reply("\nFAILURE (MISMATCH)");
                cli_abort();
            }
            else if (
                        (cli_flash_cur_off == cli_flash_exp_len)
                    &&  (cli_flash_cur_chk == cli_flash_exp_chk)
                    )
            {
                ProfileStatus const res = cli_flash_write( cli_flash_buffer, 0, cli_flash_exp_len );

                switch (res)
                {
                    case PROFILE_STATUS_SUCCESS:
                    case PROFILE_STATUS_COMMIT_SUCCESS:
                    case PROFILE_STATUS_COMMIT_SUCCESS_NOOP:
                        cli_reply("\nSUCCESS", c);
                        cli_idle();
                        break;
                    default:
                        cli_reply("\nFAILURE (WRITE)");
                        cli_abort();
                }
            }

            break;
    }
    
    cli_reply_close();

    // Required to allow subsequent receive
    cli_io_read();

    return cli_state;
}

void cli_reply( char const * p, ... )
{
    va_list va;

    va_start( va, p );

    int const len = vsprintf( (char *)&cli_reply_buffer[cli_reply_buffer_offset], p, va );

    cli_reply_buffer_offset = cli_reply_buffer_offset + len;

    va_end( va );
}

void cli_reply_scope( void )
{
    cli_lut_entry = cli_lut;
    cli_cmd = CLI_COMMAND_PROBE;

    for ( uint32_t i = 0; i < cli_lut_entries; ++i, ++cli_lut_entry )
    {
        if ((cli_lut_entry->access & CLI_ACCESS_PROBE) == CLI_ACCESS_PROBE)
        {
            cli_process_read_value = cli_process_read_type( cli_lut_entry->type );
            cli_process_read_value();
        }
    }

    cli_reply( "%s", "\r\n" );
}
