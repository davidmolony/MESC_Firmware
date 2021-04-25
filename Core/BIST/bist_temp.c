
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

    for ( float T = 10.0f; T < 80.0f; T = T + 5.0f)
    {
        uint32_t const adc = temp_get_adc( T );

        fprintf( stdout, "%2.1f 'C => ADC %03" PRIX32 "\n", T, adc );
    }

    fprintf( stdout, "Finished Temperature BIST\n" );
}
