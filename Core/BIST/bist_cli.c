
#include "MESCcli.h"

#include <inttypes.h>
#include <stdio.h>

static char const * commands[] =
{
    "",             // Empty line
    "W i 1",        // Ideal write command
    "R i",          // Ideal read line
    "W u 5 ",       // Permitted write line
    "R u ",         // Permitted read line

    "W i -5",       // int
    "R i",

    "W u 7",        // uint
    "R u",

    "W f -2.3",     // float
    "R f",
    "W f +45.67",
    "R f",

    "X reset",      // function execution

    "I i 10",       // Increase
    "R i",

    "D f 12.5",     // Decrease
    "R f",
};

static int32_t  i;
static uint32_t u;
static float    f;

static void reset( void )
{
    fprintf( stdout, ">>> RESET <<<\n" );
}

void bist_cli( void )
{
    fprintf( stdout, "Starting CLI BIST\n" );

    cli_register_variable_rw( "i", &i, sizeof(i), CLI_VARIABLE_INT   );
    cli_register_variable_rw( "u", &u, sizeof(u), CLI_VARIABLE_UINT  );
    cli_register_variable_rw( "f", &f, sizeof(f), CLI_VARIABLE_FLOAT );
    cli_register_function(  "reset", reset );

    for ( uint32_t cmd = 0; cmd < (sizeof(commands) / sizeof(commands[0])); ++cmd )
    {
        fprintf( stdout, "Processing command line '%s'\n", commands[cmd] );

        char const * p = commands[cmd];

        // Process command string
        for ( uint32_t chr = 0; (p[chr] != '\0') ; ++chr )
        {
            char const c = p[chr];

            cli_process( c );
        }

        // Commit command line
        cli_process( '\n' );
    }

    fprintf( stdout, "Finished CLI BIST\n" );
}
