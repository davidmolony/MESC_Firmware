
#include "MESCcli.h"

#include <stdio.h>

static char const * commands[] =
{
    "",             // Empty line
    "W police 1",   // Ideal write command
    "R police",     // Ideal read line
    "W police 5 ",  // Permitted write line
    "R police ",    // Permitted read line
};

static uint8_t police = 0xFF;

void bist_cli( void )
{
    fprintf( stdout, "Starting CLI BIST\n" );

    cli_register_variable_rw( "police", &police, sizeof(police), cli_process_bool, NULL );

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
