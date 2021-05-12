/*
* Copyright 2021 cod3b453
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "MESCtemp.h"
#include "util_ntc.h"

#include "conversions.h"

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
        return -999.9f;
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

    return 999.9f;
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

static float const Steinhart_Hart_T0 = CVT_CELSIUS_TO_KELVIN_F( 25.0f ); // Profile (from NTC spec)
static float const Steinhart_Hart_R0 = 10000.0f; // Profile (from NTC spec)
static float const Steinhart_Hart_Beta = 3437.864258f; // Profile (from NTC spec) OR derive from curve (util_ntc)
static float const Steinhart_Hart_r = 0.098243f; // Derive from NTC spec

#if 0 // C > 0
static float const Steinhart_Hart_A = 0.000674f; // Derived from curve (util_ntc)
static float const Steinhart_Hart_B = 0.000291f; // Derived from curve (util_ntc)
static float const Steinhart_Hart_C = 0.000000f; // Derived from curve (util_ntc)
#endif

static float Steinhart_Hart( float const R_T )
{
#if 0 // ABC
#if 0 // C > 0
    return (1.0f / (Steinhart_Hart_A + (Steinhart_Hart_B * logf( R_T )) + (Steinhart_Hart_C * logf( R_T ) * logf( R_T ) * logf( R_T ))));
#else
    return (1.0f / (Steinhart_Hart_A + (Steinhart_Hart_B * logf( R_T ))));
#endif
#else // Beta r
    return Steinhart_Hart_Beta / logf( R_T / Steinhart_Hart_r );
#endif
}

static float ntc_read_SH( uint32_t const adc_raw )
{
    float const adc  = (float)adc_raw;
    float const Vout = ((TEMP_V * adc) / adc_range);
    float const R_T = temp_calculate_R_T( Vout );

    float const K = Steinhart_Hart( R_T );

    float const T = CVT_KELVIN_TO_CELSIUS_F( K );

    return T;
}

static uint32_t SH_calculate( float const T )
{
    float const K = CVT_CELSIUS_TO_KELVIN_F( T );
#if 0 // C > 0
    float const x = (Steinhart_Hart_A - (1.0f / K)) / Steinhart_Hart_C;

    float const x_d_2_p2 = (x / 2.0f)
                         * (x / 2.0f);

    float const B_d_3C_p3 = (Steinhart_Hart_B / (3.0f * Steinhart_Hart_C))
                          * (Steinhart_Hart_B / (3.0f * Steinhart_Hart_C))
                          * (Steinhart_Hart_B / (3.0f * Steinhart_Hart_C));

    float const y = sqrtf( B_d_3C_p3 + x_d_2_p2 );

    float const R_T = expf( pow( (y - (x / 2.0f)), (1.0f / 3.0f) ) - pow( (y + (x / 2.0f)), (1.0f / 3.0f) ) );
#else
    float const R_T = Steinhart_Hart_R0 * expf( Steinhart_Hart_Beta * ( (1.0f / K) - (1.0f / Steinhart_Hart_T0) ) );
#endif

    float const Vout = (TEMP_V * R_T) / (TEMP_R_F + R_T);
    uint32_t const adc_raw = (uint32_t)((Vout * ((float)adc_range)) / TEMP_V);

    return adc_raw;
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

    uint32_t const adc_granularity = UINT32_C(128);
    uint32_t const adc_mask        = (adc_granularity - 1);

    uint32_t adc_min = temp_get_adc( Tmin );
    uint32_t adc_max = temp_get_adc( Tmax );

    uint32_t adc_start;
    uint32_t adc_end;

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

    for ( uint32_t adc = adc_start; adc <= adc_end; adc = adc + adc_granularity )
    {
        float const Tapx = temp_read( adc );
        float const Tact = ntc_read( adc );
        float const Tsh = ntc_read_SH( adc );

        fprintf( stdout, "ADC %03" PRIX32 " => %5.1f 'C (%5.1f 'C) %5.1f 'C\n", adc, Tapx, Tact, Tsh );
    }

    for ( float T = 0.0f; T <= 100.0f; T = T + 5.0f)
    {
        uint32_t const adc = temp_get_adc( T );
        uint32_t const adc_act = adc_calculate( T );
        uint32_t const adc_SH = SH_calculate( T );

        fprintf( stdout, "%5.1f 'C => ADC %03" PRIX32 " (%03" PRIX32 ") %03" PRIX32 "\n", T, adc, adc_act, adc_SH );
    }

    fprintf( stdout, "Finished Temperature BIST\n" );
}
