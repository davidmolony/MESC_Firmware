
#include "MESCcli.h"
#include "MESCfnv.h"

//#include <inttypes.h>//debug
#include <math.h>
#include <stddef.h>
//#include <stdio.h>//debug
#include <string.h>

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

    CLI_ACCESS_RO   = CLI_ACCESS_R,
    CLI_ACCESS_WO   = CLI_ACCESS_W,
    CLI_ACCESS_RW   = (CLI_ACCESS_R | CLI_ACCESS_W),
};

typedef enum CLIAccess CLIAccess;

struct CLIVar
{
    uint32_t        state;
    union
    {
    int32_t         i;
    uint32_t        u;
    float           f;
    };
};

typedef struct CLIVar CLIVar;

static CLIVar cli_var = {0};

struct CLIEntry
{
    uint32_t        hash;
    union
    {
    void       *    wvar;
    void const *    rvar;
    };
    uint32_t        size;
    CLIAccess       access;
    void (*         fn)( char const );
};

typedef struct CLIEntry CLIEntry;

static CLIEntry cli_lut[32];
static uint32_t cli_lut_entries = 0;
static CLIEntry * cli_lut_entry = NULL;

static void cli_noop( char const c )
{
    cli_state = CLI_STATE_ABORT;
    cli_hash_valid = 0;
//fprintf( stderr, "WARNING: cli_noop (%c)\n", c );//debug
    (void)c;
}

static void (* cli_process_value)( char const ) = cli_noop;

static void cli_execute( void )
{
//fprintf( stderr, "INFO: Executing '%c'\n", cli_cmd );//debug
    switch (cli_cmd)
    {
        case 'R':
            // read and echo or error
//fprintf( stderr, "INFO: Read %" PRIu32 " bytes [ ", cli_lut_entry->size );//debug
//            for ( uint32_t i = 0; i < cli_lut_entry->size; i = i + 1)//debug
//            {//debug
//fprintf( stderr, "%02" PRIX8 " ", *((uint8_t const *)cli_lut_entry->rvar) );//debug
//            }//debug
//fprintf( stderr, "]\n" );//debug
            break;
        case 'W':
            // write or error
            memcpy( cli_lut_entry->wvar, &cli_var.u, cli_lut_entry->size );
//fprintf( stderr, "INFO: Commit %" PRIu32 " bytes\n", cli_lut_entry->size );//debug
            break;
        default:
            // error
//fprintf( stderr, "ERROR: cli_execute failed (%02" PRIX8 ")\n", cli_cmd );//debug
            break;
    }

    cli_hash_valid = 0;
    cli_state = CLI_STATE_IDLE;
    cli_var.state = 0;
}

static CLIEntry * cli_lut_alloc( char const * name )
{
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
//fprintf( stderr, "INFO: Registered '%s' as %08" PRIX32 "\n", name, hash );//debug
    return entry;
}

void cli_process_int( char const c )
{
    if ((cli_var.state == 0) && (c == '-'))
    {
        cli_var.i = INT32_C(-1);
        cli_var.state = 1;
    }

    if (('0' <= c) && (c <= '9'))
    {
        if (cli_var.state == 0)
        {
            cli_var.i = INT32_C(0);
        }
        cli_var.i *= INT32_C(10);
        cli_var.i += (c - '0');
        cli_var.state = 1;
    }
    else if (cli_var.state == 0)
    {
        cli_state = CLI_STATE_ABORT;
    }
}

void cli_process_uint( char const c )
{
    if (('0' <= c) && (c <= '9'))
    {
        if (cli_var.state == 0)
        {
            cli_var.u = UINT32_C(0);
        }
        cli_var.u *= UINT32_C(10);
        cli_var.u += (c - '0');
        cli_var.state = 1;
    }
    else
    {
        cli_state = CLI_STATE_ABORT;
    }
}

void cli_process_float( char const c )
{
    switch (cli_var.state)
    {
        case 0:
            cli_var.state = 1;
            switch (c)
            {
                case '-':
                    cli_var.f = -1.0f;
                    break;
                case '+':
                    cli_var.f = +1.0f;
                    break;
                default:
                    if (('0' <= c) && (c <= '9'))
                    {
                        cli_var.f = ((float)(c - '0'));
                        cli_var.state = 2;
                    }
                    else
                    {
                        cli_state = CLI_STATE_ABORT;
                    }
                    break;
            }
            break;
        case 1:
            cli_var.state = 2;
            if (('0' <= c) && (c <= '9'))
            {
                cli_var.f *= ((float)(c - '0'));
            }
            else
            {
                cli_state = CLI_STATE_ABORT;
            }
            break;
        case 2:
            if (('0' <= c) && (c <= '9'))
            {
                float const sgn = (cli_var.f < 0.0f) ? -1.0f : 1.0f;
                cli_var.f *= 10.0f;
                cli_var.f += sgn * ((float)(c - '0'));
            }
            else if (c == '.')
            {
                cli_var.state = 3;
            }
            else
            {
                cli_state = CLI_STATE_ABORT;
            }
            break;
        default:
            if (('0' <= c) && (c <= '9'))
            {
                float const sgn = (cli_var.f < 0.0f) ? -1.0f : 1.0f;
                cli_var.f += (sgn * ((float)(c - '0'))) / pow( 10.0f, (cli_var.state - 2)  );
                cli_var.state++;
            }
            else
            {
                cli_state = CLI_STATE_ABORT;
            }
            break;
    }
}

void cli_register_variable_ro(
    char const * name,
    void const * address, uint32_t const size,
    void (* fn)( char const c ) )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->rvar   = address;
        entry->size   = size;
        entry->access = CLI_ACCESS_RO;
        entry->fn     = fn;

        cli_lut_entries++;
    }
}

void cli_register_variable_rw(
    char const * name,
    void       * address, uint32_t const size,
    void (* fn)( char const c) )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->wvar   = address;
        entry->size   = size;
        entry->access = CLI_ACCESS_RW;
        entry->fn     = fn;

        cli_lut_entries++;
    }
}

void cli_register_variable_wo(
    char const * name,
    void       * address, uint32_t const size,
    void (* fn)( char const c ) )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->wvar   = address;
        entry->size   = size;
        entry->access = CLI_ACCESS_WO;
        entry->fn     = fn;

        cli_lut_entries++;
    }
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

static void cli_process_variable( const char c )
{
    switch (c)
    {
        case '\n':
            if (cli_hash_valid != 0)
            {
                cli_execute();
            }
            break;
        case ' ':
            cli_lut_entry = cli_lookup( cli_hash );
            cli_hash_valid = 0;
            {
            CLIAccess access = CLI_ACCESS_NONE;
            switch (cli_cmd)
            {
                case 'R':
                    access = CLI_ACCESS_R;
                    break;
                case 'W':
                    access = CLI_ACCESS_W;
                    break;
            }

            if ((cli_lut_entry == NULL) || ((cli_lut_entry->access & access) != access))
            {
                cli_state = CLI_STATE_ABORT;
                cli_process_value = cli_noop;
                break;
            }
            }
            switch (cli_cmd)
            {
                case 'R':
                    cli_state = CLI_STATE_EXECUTE;
                    break;
                case 'W':
                    cli_state = CLI_STATE_VALUE;
                    cli_process_value = cli_lut_entry->fn;
                    break;
            }
            break;
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
                    cli_state = CLI_STATE_IDLE;
                    cli_hash_valid = 0;
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
                    cli_state = CLI_STATE_IDLE;
                    cli_hash_valid = 0;
                    cli_var.state = 0;
                }
                else
                {
                    cli_state = CLI_STATE_ABORT;
                    cli_hash_valid = 0;
                }
            }
//fprintf( stderr, "HASH %08" PRIX32 "\n", cli_hash );//debug
            break;
    }
}

void cli_process( char const c )
{
    switch (cli_state)
    {
        case CLI_STATE_IDLE:
            switch (c)
            {
                case '\n':
                    break;
                case 'R':
                case 'W':
                    cli_cmd = c;
                    cli_state = CLI_STATE_COMMAND;
                    break;
                default:
                    cli_state = CLI_STATE_ABORT;
                    cli_hash_valid = 0;
                    break;
            }
            break;
        case CLI_STATE_ABORT:
            if (c == '\n')
            {
                cli_state = CLI_STATE_IDLE;
                cli_var.state = 0;
            }
            break;
        case CLI_STATE_COMMAND:
            switch (c)
            {
                case '\n':
                    cli_state = CLI_STATE_IDLE;
                    cli_var.state = 0;
                    break;
                case ' ':
                    cli_state = CLI_STATE_VARIABLE;
                    break;
                default:
                    cli_state = CLI_STATE_ABORT;
                    cli_hash_valid = 0;
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
                cli_process_value( c );
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
                    cli_state = CLI_STATE_ABORT;
                    cli_hash_valid = 0;
                    break;
            }
            break;
    }
}
