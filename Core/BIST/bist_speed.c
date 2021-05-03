
#include "MESCspeed.h"

#include <stdio.h>

void bist_speed( void )
{
    fprintf( stdout, "Starting Speed BIST\n" );

    speed_set_motor( 6 );
    speed_set_wheel( 26.0f ); // Inches
    speed_set_units( 63360.0f ); // Inches per Mile
    speed_set_gear_ratio( 1, 1 );

    speed_get();

    fprintf( stdout, "Finished Speed BIST\n" );
}
