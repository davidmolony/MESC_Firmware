
#include "MESCtemp.h"

#include <inttypes.h>
#include <stdio.h>

void bist_temp( void )
{
    fprintf( stdout, "Starting Temperature BIST\n" );

    for ( uint16_t adc = 0; adc < 4096; adc = adc + 16 )
    {
        float const T = temp_read( adc );

        fprintf( stdout, "ADC %03" PRIX32 " => %2.1f 'C\n", adc, T );
    }

    fprintf( stdout, "Finished Temperature BIST\n" );
}
