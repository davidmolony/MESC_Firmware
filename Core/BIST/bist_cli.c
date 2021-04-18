
#include "MESCcli.h"

#include <stdio.h>

static char const * commands[] =
{
    "\n",               // Empty line
    "W police 1\n",     // Ideal write command
    "W police 1 \n",    // Permitted write line
    "R police\n",       // Ideal read line
    "R police \n",      // Permitted read line
};

void bist_cli( void )
{
    fprintf( stdout, "Starting CLI BIST\n" );

    for ( uint32_t cmd = 0; cmd < (sizeof(commands) / sizeof(commands[0])); ++cmd )
    {
        char const * p = commands[cmd];

        for ( uint32_t chr = 0; (p[chr] != '\0') ; ++chr )
        {
            char const c = p[chr];

            cli_process( c );
        }
    }

    fprintf( stdout, "Finished CLI BIST\n" );
}
