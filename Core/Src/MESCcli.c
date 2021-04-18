
#include "MESCcli.h"
#include "MESCfnv.h"

#include <stddef.h>

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

static void cli_execute( void )
{
    switch (cli_cmd)
    {
        case 'R':
            // read and echo or error
            break;
        case 'W':
            // write or error
            break;
        default:
            // error
            break;
    }

    cli_state = CLI_STATE_IDLE;
}
#if 0 // todo
static uint32_t cli_value = 0;

static void cli_process_bool( char const c )
{
    switch (cli_state)
    {
        case CLI_STATE_VALUE:
            switch (c)
            {
                case ' ':
                    return;
                case '0':
                    cli_value = 0;
                    cli_state = CLI_STATE_EXECUTE;
                    break;
                case '1':
                    cli_value = 1;
                    cli_state = CLI_STATE_EXECUTE;
                    break;
                default:
                    cli_state = CLI_STATE_ABORT;
                    break;
            }
            break;
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
                    break;
            }
            break;
        default:
            cli_state = CLI_STATE_ABORT;
            break;
    }
}
#endif
static void cli_noop( char const c )
{
    cli_state = CLI_STATE_ABORT;
    (void)c;
}

static void (* cli_process_value)( char const ) = cli_noop;

static uint8_t cli_hash_valid = 0;
static uint32_t cli_hash = 0;

struct CLIVar
{
    uint32_t hash;
    uint32_t size;
    void *   ptr;
    uint32_t access;
};

void cli_register_variable_ro( char const * name, void const * address, uint32_t const size )
{
    (void)name;
    (void)address;
    (void)size;
}

void cli_register_variable_rw( char const * name, void       * address, uint32_t const size )
{
    (void)name;
    (void)address;
    (void)size;
}

void cli_register_variable_wo( char const * name, void       * address, uint32_t const size )
{
    (void)name;
    (void)address;
    (void)size;
}

static void (* cli_lookup( uint32_t const hash ))( const char )
{
    (void)hash;
    return cli_noop;
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
            switch (cli_cmd)
            {
                case 'R':
                    cli_state = CLI_STATE_EXECUTE;
                    break;
                case 'W':
                    cli_state = CLI_STATE_VALUE;
                    cli_process_value = cli_lookup( cli_hash );
                    break;
            }
            break;
        default:
            if (cli_hash_valid != 0)
            {
                if  (
                        (('0' <= c ) && (c <= '1'))
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
                    cli_hash_valid = 1;
                }
                else if (c == '\n')
                {
                    cli_state = CLI_STATE_IDLE;
                }
                else
                {
                    cli_state = CLI_STATE_ABORT;
                }
            }
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
                    break;
            }
            break;
        case CLI_STATE_ABORT:
            if (c == '\n')
            {
                cli_state = CLI_STATE_IDLE;
            }
            break;
        case CLI_STATE_COMMAND:
            switch (c)
            {
                case '\n':
                    cli_state = CLI_STATE_IDLE;
                    break;
                case ' ':
                    cli_state = CLI_STATE_VARIABLE;
                    break;
                default:
                    cli_state = CLI_STATE_ABORT;
                    break;
            }
            break;
        case CLI_STATE_VARIABLE:
            cli_process_variable( c );
            break;
        case CLI_STATE_VALUE:
            cli_process_value( c );
            break;
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
                    break;
            }
            break;
    }
}
