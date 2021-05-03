
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

    "W u 7",        // uint

    "W f -2.3",     // float
    "W f +45.67",
};

static int32_t  i;
static uint32_t u;
static float    f;

void bist_cli( void )
{
    fprintf( stdout, "Starting CLI BIST\n" );

    cli_register_variable_rw( "i", &i, sizeof(i), cli_process_int   );
    cli_register_variable_rw( "u", &u, sizeof(u), cli_process_uint  );
    cli_register_variable_rw( "f", &f, sizeof(f), cli_process_float );

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

        fprintf( stdout, "i %" PRId32 " "
                         "u %" PRIu32 " "
                         "f %f\n",
                         i, u, f );
    }

    fprintf( stdout, "Finished CLI BIST\n" );
}
