
#include "MESCtemp.h"
#include "util_ntc.h"

#include <assert.h>
#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static void ntc_T_minmax( float * Tmin, float * Tmax)
{
    *Tmin =  FLT_MAX;
    *Tmax = -FLT_MAX;

    for ( size_t i = 0; i < ntc_T_R_count; ++i )
    {
        if (ntc_T_R[i].T < *Tmin)
        {
            *Tmin = ntc_T_R[i].T;
        }

        if (ntc_T_R[i].T > *Tmax)
        {
            *Tmax = ntc_T_R[i].T;
        }
    }

    assert( *Tmin !=  FLT_MAX );
    assert( *Tmax != -FLT_MAX );
}

static float ntc_calculate( float const R_T )
{
    if (R_T > ntc_T_R[0].R)
    {
        return -FLT_MAX;
    }

    for ( size_t i = 0; i < ntc_T_R_count; )
    {
        NTCNode const * lo = &ntc_T_R[i];
        i = i + 1;
        NTCNode const * hi = &ntc_T_R[i];

        if  (// DANGER - NTC resistance is inverted
                (hi->R <= R_T)
            &&  (R_T <= lo->R)
            )
        {
            float const dT = (hi->T - lo->T);
            float const dR = (lo->R - hi->R);
            float const nR_T = (lo->R - R_T);

            float const T = (lo->T + ((dT * nR_T) / dR));

            return T;
        }
    }

    return FLT_MAX;
}

#define TEMP_V   (3.3f)     // Volts
#define TEMP_R_F (4700.0f)  // Ohms

/*
Schematic

 -+- V
  |
 | | R_F = 4k7
 |_|
  |
  +- Vout - >ADC
 \|
 |\| R_T
 |_\_
  |
 -+-

R_T = Vout * R_F
      ----------
      (V - Vout)
*/

static float temp_calculate_R_T( float const Vout )
{
    float const num = (Vout * TEMP_R_F);
    float const den = (TEMP_V - Vout);
    float const R_T = (num / den);

    return R_T;
}

static uint32_t adc_range = UINT32_C(4096);    // Profile

static float ntc_read( uint32_t const adc_raw )
{
    float const adc  = (float)adc_raw;
    float const Vout = ((TEMP_V * adc) / adc_range);
    float const R_T = temp_calculate_R_T( Vout );

    float const T = ntc_calculate( R_T );

    return T;
}

static uint32_t adc_calculate( float const T )
{
    if (T < ntc_T_R[0].T)
    {
        return (adc_range - 1);
    }

    for ( size_t i = 0; i < ntc_T_R_count; )
    {
        NTCNode const * lo = &ntc_T_R[i];
        i = i + 1;
        NTCNode const * hi = &ntc_T_R[i];

        if  (
                (lo->T <= T)
            &&  (T <= hi->T)
            )
        {
            float const dT = (hi->T - lo->T);
            float const dR = (hi->R - lo->R);

            float const R_T = (lo->R + ((dR * (T - lo->T)) / dT));

            float const adc_range_ = (float)(adc_range - 1);
            float const adc_ = ((adc_range_ * R_T) / (TEMP_R_F + R_T));

            uint32_t const adc = (uint32_t)(adc_);

            return adc;
        }
    }

    return 0;
}

void bist_temp( void )
{
    fprintf( stdout, "Starting Temperature BIST\n" );

    float Tmin;
    float Tmax;

    ntc_T_minmax( &Tmin, &Tmax );

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
        adc_max &= ~adc_mask;

        adc_start = adc_min;
        adc_end   = adc_max;
    }
    else
    {
        adc_min +=  adc_mask;
        adc_min &= ~adc_mask;
        adc_max +=  adc_mask;
        adc_max &= ~adc_mask;

        adc_start = adc_max;
        adc_end   = adc_min;
    }

    for ( uint16_t adc = adc_start; adc <= adc_end; adc = adc + adc_granularity )
    {
        float const Tapx = temp_read( adc );
        float const Tact = ntc_read( adc );

        fprintf( stdout, "ADC %03" PRIX32 " => %5.1f 'C (%5.1f 'C)\n", adc, Tapx, Tact );
    }

    for ( float T = 0.0f; T <= 100.0f; T = T + 5.0f)
    {
        uint32_t const adc = temp_get_adc( T );
        uint32_t const adc_act = adc_calculate( T );

        fprintf( stdout, "%5.1f 'C => ADC %03" PRIX32 " (%03" PRIX32 ")\n", T, adc, adc_act );
    }

    fprintf( stdout, "Finished Temperature BIST\n" );
}
