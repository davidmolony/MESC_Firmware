
#include "MESCcli.h"
#include "MESCfnv.h"

//#include <inttypes.h>//debug
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

struct CLIEntry
{
    uint32_t        hash;
    union
    {
    void       *    wvar;
    void const *    rvar;
    };
    uint32_t        size;
    uint8_t         access;
    void (*         fn)( void const *, char const );
    void *          arg;
};

typedef struct CLIEntry CLIEntry;

static CLIEntry cli_lut[32];
static uint32_t cli_lut_entries = 0;
static CLIEntry * cli_lut_entry = NULL;

static void cli_noop( void const * entry, char const c )
{
    cli_state = CLI_STATE_ABORT;
    cli_hash_valid = 0;
//fprintf( stderr, "WARNING: cli_noop (%c)\n", c );//debug
    (void)entry;
    (void)c;
}

static void (* cli_process_value)( void const *, char const ) = cli_noop;

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
            memcpy( cli_lut_entry->wvar, &cli_lut_entry->arg, cli_lut_entry->size );
//fprintf( stderr, "INFO: Commit %" PRIu32 " bytes\n", cli_lut_entry->size );//debug
            break;
        default:
            // error
//fprintf( stderr, "ERROR: cli_execute failed (%02" PRIX8 ")\n", cli_cmd );//debug
            break;
    }

    cli_hash_valid = 0;
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
                    cli_hash_valid = 0;
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
                    cli_hash_valid = 0;
                    break;
            }
            break;
        default:
            cli_state = CLI_STATE_ABORT;
            cli_hash_valid = 0;
            break;
    }
}
#endif

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

void cli_process_bool( void const * entry, char const c )
{
    CLIEntry const * cli_entry = (CLIEntry const *)entry;
    // TODO check cli_entry
    // TODO check command
    // TODO check access
    switch (cli_cmd)
    {
        case 'R':
            cli_state = CLI_STATE_EXECUTE;
            break;
        case 'W':
//fprintf( stderr, "INFO: Write bool '%c'\n", c );//debug
            *((uintptr_t *)&cli_entry->arg) |= (c ? 1 : 0);
            cli_state = CLI_STATE_EXECUTE;
            break;
        default:
            cli_state = CLI_STATE_ABORT;
            cli_hash_valid = 0;
            break;
    }
}

void cli_register_variable_ro( char const * name, void const * address, uint32_t const size, void (* fn)( void const * entry, char const c ), void * info )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->rvar   = address;
        entry->size   = size;
        entry->access = 0x1; // TODO
        entry->fn     = fn;
        entry->arg    = info;

        cli_lut_entries++;
    }
}

void cli_register_variable_rw( char const * name, void       * address, uint32_t const size, void (* fn)( void const * entry, char const c), void * info )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->wvar = address;
        entry->size = size;
        entry->access = 0x3; // TODO
        entry->fn     = fn;
        entry->arg    = info;

        cli_lut_entries++;
    }
}

void cli_register_variable_wo( char const * name, void       * address, uint32_t const size, void (* fn)( void const * entry, char const c ), void * info )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->wvar = address;
        entry->size = size;
        entry->access = 0x2; // TODO
        entry->fn     = fn;
        entry->arg    = info;

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
            switch (cli_cmd)
            {
                case 'R':
                    cli_state = CLI_STATE_EXECUTE;
                    break;
                case 'W':
                    cli_state = CLI_STATE_VALUE;
                    cli_lut_entry = cli_lookup( cli_hash );
                    if (cli_lut_entry == NULL)
                    {
                        cli_state = CLI_STATE_ABORT;
                        cli_hash_valid = 0;
                        cli_process_value = cli_noop;
                    }
                    else
                    {
                        cli_hash_valid = 0;
                        cli_process_value = cli_lut_entry->fn;
                    }
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
                    cli_hash_valid = 0;
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
                    cli_hash_valid = 0;
                    break;
            }
            break;
        case CLI_STATE_VARIABLE:
            cli_process_variable( c );
            break;
        case CLI_STATE_VALUE:
            cli_process_value( cli_lut_entry, c );
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
                    cli_hash_valid = 0;
                    break;
            }
            break;
    }
}
