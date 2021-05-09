
#include "MESCtemp.h"

#include <inttypes.h>
#include <stdio.h>

void bist_temp( void )
{
    fprintf( stdout, "Starting Temperature BIST\n" );

    float const Tmin = -20.0f;
    float const Tmax = 120.0f;

    uint16_t const adc_granularity = UINT16_C(128);
    uint16_t const adc_mask        = (adc_granularity - 1);

    uint16_t adc_min = temp_get_adc( Tmin );
    uint16_t adc_max = temp_get_adc( Tmax );

    uint16_t adc_start;
    uint16_t adc_end;

    if (adc_min < adc_max)
    {
        adc_min &= ~adc_mask;
        adc_max +=  adc_mask;
        adc_min &= ~adc_mask;

        adc_start = adc_min;
        adc_end   = adc_max;
    }
    else
    {
        adc_min +=  adc_mask;
        adc_min &= ~adc_mask;
        adc_max &= ~adc_mask;

        adc_start = adc_max;
        adc_end   = adc_min;
    }

    for ( uint16_t adc = adc_start; adc < adc_end; adc = adc + adc_granularity )
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
