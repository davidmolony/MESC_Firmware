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

#include "MESCcli.h"
#include "MESCfnv.h"

#include "pp_op.h"

#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define MAKE_TYPE_SIZE(type,size)      ((uint32_t)((uint32_t)((type) << 4) | ((uint32_t)(size))))
#define MAKE_TYPE_SIZE_CASE(type,size) ((uint32_t)((uint32_t)((JOIN( CLI_VARIABLE_, type)) << 4) | ((uint32_t)(size))))

enum CLIState
{
    CLI_STATE_IDLE,

    CLI_STATE_ABORT,

    CLI_STATE_COMMAND,
    CLI_STATE_VARIABLE,
    CLI_STATE_VALUE,

    CLI_STATE_EXECUTE,
};

typedef enum CLIState CLIState;

static CLIState cli_state;
static uint8_t cli_cmd;

static uint8_t cli_hash_valid = 0;
static uint32_t cli_hash = 0;

enum CLIAccess
{
    CLI_ACCESS_NONE = 0x0,

    CLI_ACCESS_R    = 0x1,
    CLI_ACCESS_W    = 0x2,
    CLI_ACCESS_X    = 0x4,

    CLI_ACCESS_RO   = CLI_ACCESS_R,
    CLI_ACCESS_WO   = CLI_ACCESS_W,
    CLI_ACCESS_RW   = (CLI_ACCESS_R | CLI_ACCESS_W),
};

typedef enum CLIAccess CLIAccess;

struct CLIVar
{
    uint32_t        state; // TODO enum
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
static CLIEntry cli_lut[MAX_CLI_LUT_ENTRIES];
static uint32_t cli_lut_entries = 0;
static CLIEntry * cli_lut_entry = NULL;

static int cli_io_write_noop( void * handle, void * data, uint16_t size, uint32_t timeout )
{
    (void)handle;
    (void)data;
    (void)size;
    (void)timeout;

    return 0;
}

static void * cli_io_handle = NULL;
static int (* cli_io_write)( void *, void *, uint16_t, uint32_t ) = cli_io_write_noop;

static void cli_idle( void )
{
    cli_state = CLI_STATE_IDLE;
    cli_hash_valid = 0;
    cli_hash = 0;
}

static void cli_abort( void )
{
    cli_state = CLI_STATE_ABORT;
    cli_hash_valid = 0;
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

static void cli_execute( void )
{
    switch (cli_cmd)
    {
        case 'R':
            cli_process_read_value();
            break;
        case 'W':
            memcpy( cli_lut_entry->var.w, &cli_var.var, cli_lut_entry->size );
            break;
        case 'X':
            cli_lut_entry->var.x();
            break;
        case 'I':
        case 'D':
        {
            CLIVariableType const type = cli_lut_entry->type;
            uint32_t const size = cli_lut_entry->size;
            union
            {
                int8_t  i8;
                uint8_t u8;

                int16_t  i16;
                uint16_t u16;

                int32_t  i32;
                uint32_t u32;

                float    f32;
            } * tmp = cli_lut_entry->var.w;

            switch (MAKE_TYPE_SIZE( type, size ))
            {
                case MAKE_TYPE_SIZE_CASE( INT, 1 ):
                    if (cli_cmd == 'I')
                    {
                        tmp->i8 += (int8_t)cli_var.var.i;
                    }
                    else
                    {
                        tmp->i8 -= (int8_t)cli_var.var.i;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( INT, 2 ):
                    if (cli_cmd == 'I')
                    {
                        tmp->i16 += (int16_t)cli_var.var.i;
                    }
                    else
                    {
                        tmp->i16 -= (int16_t)cli_var.var.i;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( INT, 4 ):
                    if (cli_cmd == 'I')
                    {
                        tmp->i32 += (int32_t)cli_var.var.i;
                    }
                    else
                    {
                        tmp->i32 -= (int32_t)cli_var.var.i;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( UINT, 1 ):
                    if (cli_cmd == 'I')
                    {
                        tmp->u8 += (uint8_t)cli_var.var.u;
                    }
                    else
                    {
                        tmp->u8 -= (uint8_t)cli_var.var.u;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( UINT, 2 ):
                    if (cli_cmd == 'I')
                    {
                        tmp->u16 += (uint16_t)cli_var.var.u;
                    }
                    else
                    {
                        tmp->u16 -= (uint16_t)cli_var.var.u;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( UINT, 4 ):
                    if (cli_cmd == 'I')
                    {
                        tmp->u32 += (uint32_t)cli_var.var.u;
                    }
                    else
                    {
                        tmp->u32 -= (uint32_t)cli_var.var.u;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( FLOAT, 4 ):
                    if (cli_cmd == 'I')
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
        default:
            // error
            break;
    }

    cli_idle();
    cli_var.state = 0;
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
        uint32_t const size = cli_lut_entry->size;
        char * fmt_10 = NULL;
        char * fmt_16 = NULL;

        switch (MAKE_TYPE_SIZE( type, size ))
        {
            case MAKE_TYPE_SIZE_CASE( INT, 1 ):
                fmt_10 = PRId8;
                fmt_16 = PRIX8;
                break;
            case MAKE_TYPE_SIZE_CASE( INT, 2 ):
                fmt_10 = PRId16;
                fmt_16 = PRIX16;
                break;
            case MAKE_TYPE_SIZE_CASE( INT, 4 ):
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

        uint32_t const nybbles = UINT32_C(2) * size;

        sprintf( cli_buffer,
            "%%" "%s"
            " 0x" "%%" "0%" PRIu32 "%s"
            "\r" "\n",
            fmt_10,
            nybbles, fmt_16 );

        uint32_t v = UINT32_C(0);

        memcpy( &v, cli_lut_entry->var.r, size );

        cli_reply( cli_buffer, v, v );
    }
}

static void cli_process_write_int( char const c )
{
    if ((cli_var.state == 0) && (c == '-'))
    {
        cli_var.var.i = INT32_C(-1);
        cli_var.state = 1;
    }

    if (('0' <= c) && (c <= '9'))
    {
        if (cli_var.state == 0)
        {
            cli_var.var.i = INT32_C(0);
        }
        cli_var.var.i *= INT32_C(10);
        cli_var.var.i += (c - '0');
        cli_var.state = 1;
    }
    else if (cli_var.state == 0)
    {
        cli_abort();
    }
}

static void cli_process_write_uint( char const c )
{
    if (('0' <= c) && (c <= '9'))
    {
        if (cli_var.state == 0)
        {
            cli_var.var.u = UINT32_C(0);
        }
        cli_var.var.u *= UINT32_C(10);
        cli_var.var.u += (c - '0');
        cli_var.state = 1;
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
        uint32_t const size = cli_lut_entry->size;

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
    switch (cli_var.state)
    {
        case 0:
            cli_var.state = 1;
            switch (c)
            {
                case '-':
                    cli_var.var.f = -1.0f;
                    break;
                case '+':
                    cli_var.var.f = +1.0f;
                    break;
                default:
                    if (('0' <= c) && (c <= '9'))
                    {
                        cli_var.var.f = ((float)(c - '0'));
                        cli_var.state = 2;
                    }
                    else
                    {
                        cli_abort();
                    }
                    break;
            }
            break;
        case 1:
            cli_var.state = 2;
            if (('0' <= c) && (c <= '9'))
            {
                cli_var.var.f *= ((float)(c - '0'));
            }
            else
            {
                cli_abort();
            }
            break;
        case 2:
            if (('0' <= c) && (c <= '9'))
            {
                float const sgn = (cli_var.var.f < 0.0f) ? -1.0f : 1.0f;
                cli_var.var.f *= 10.0f;
                cli_var.var.f += sgn * ((float)(c - '0'));
            }
            else if (c == '.')
            {
                cli_var.state = 3;
            }
            else
            {
                cli_abort();
            }
            break;
        default:
            if (('0' <= c) && (c <= '9'))
            {
                float const sgn = (cli_var.var.f < 0.0f) ? -1.0f : 1.0f;
                cli_var.var.f += (sgn * ((float)(c - '0'))) / (float)pow( 10.0f, (float)(cli_var.state - 2)  );
                cli_var.state++;
            }
            else
            {
                cli_abort();
            }
            break;
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
    void * handle,
    int (* const write)( void * handle, void * data, uint16_t size, uint32_t timeout ) )
{
    cli_io_handle = handle;
    cli_io_write = write;
}

static CLIEntry * cli_lookup( uint32_t const hash )
{
    for ( uint32_t i = 0; i < cli_lut_entries; ++i )
    {
        if (cli_lut[i].hash == hash)
        {
            return &cli_lut[i];
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
                case 'R':
                    access = CLI_ACCESS_R;
                    break;
                case 'W':
                    access = CLI_ACCESS_W;
                    break;
                case 'X':
                    access = CLI_ACCESS_X;
                    break;
                case 'I':
                case 'D':
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
                case 'R':
                    cli_process_read_value = cli_process_read_type( cli_lut_entry->type );
                    // fallthrough
                case 'X':
                    cli_state = CLI_STATE_EXECUTE;

                    if  (c == '\n')
                    {
                        if (cli_hash_valid != 0)
                        {
                            cli_execute();
                        }
                        else
                        {
                            cli_abort();
                        }
                    }

                    break;
                case 'W':
                    cli_process_write_value = cli_process_write_type( cli_lut_entry->type );

                    cli_state = CLI_STATE_VALUE;
                    cli_hash_valid = 0;
                    cli_hash = 0;

                    break;
                case 'I':
                case 'D':
                    cli_process_write_value = cli_process_write_type( cli_lut_entry->type );
                    cli_process_read_value = cli_process_read_type( cli_lut_entry->type );

                    cli_state = CLI_STATE_VALUE;
                    cli_hash_valid = 0;
                    cli_hash = 0;

                    break;
                default:
                    cli_abort();
                    break;
            }
            break;
        }
        default:
            if (cli_hash_valid != 0)
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
                    cli_var.state = 0;
                }
            }
            else
            {
                if  (
                        (('A' < c ) && (c < 'Z'))
                    ||  (('a' < c ) && (c < 'z'))
                    )
                {
                    cli_hash = fnv1a_init();
                    cli_hash = fnv1a_process( cli_hash, c );
                    cli_hash_valid = 1;
                }
                else if (c == '\n')
                {
                    cli_idle();
                    cli_var.state = 0;
                }
                else
                {
                    cli_abort();
                }
            }
//fprintf( stderr, "HASH %08" PRIX32 "\n", cli_hash );//debug
            break;
    }
}

int cli_process( char const c )
{
	if (c == '\n')
	{
		cli_reply( "%s", "\r\n" );
	}
	else
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
                case 'R':
                case 'W':
                case 'X':
                case 'I':
                case 'D':
                    cli_cmd = c;
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
                cli_var.state = 0;
            }
            break;
        case CLI_STATE_COMMAND:
            switch (c)
            {
                case '\n':
                    cli_idle();
                    cli_var.state = 0;
                    break;
                case ' ':
                    cli_state = CLI_STATE_VARIABLE;
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
    }

    return cli_state;
}

void cli_reply( char const * p, ... )
{
    uint8_t buffer[256];
    va_list va;

    va_start( va, p );

    int const len = vsprintf( (char *)buffer, p, va );

    if (len > 0)
    {
        int const ret = cli_io_write( cli_io_handle, buffer, len, (2 * len) );
        (void)ret;
    }

    va_end( va );
}
