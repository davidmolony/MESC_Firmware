/*
* Copyright 2021-2023 cod3b453
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

static float ntc_get_R_T_from_adc( TEMP const * const temp, uint32_t const adc_raw )
{
    float const adc  = (float)adc_raw;
    float const Vout = ((temp->V * adc) / (float)temp->adc_range);

    switch (temp->schema)
    {
        case TEMP_SCHEMA_R_F_ON_R_T:
        {
            float const num = (Vout * temp->R_F);
            float const den = (temp->V - Vout);
            float const R_T = (num / den);

            return R_T;
        }
        case TEMP_SCHEMA_R_T_ON_R_F:
        {
            float const num = (temp->V * temp->R_F);
            float const den = Vout;
            float const R_T = (num / den) - temp->R_F;

            return R_T;
        }
        default:
        {
            // error
            return 0.0f;
        }
    }
}

static float ntc_get_T_from_R_T( float const R_T )
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

static uint32_t ntc_get_adc_from_T( TEMP const * const temp, float const T )
{
    if (T < ntc_T_R[0].T)
    {
        return (temp->adc_range - 1);
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

            float const adc_range_ = (float)(temp->adc_range - 1);

            float adc_ = 0.0f;

            switch (temp->schema)
            {
                case TEMP_SCHEMA_R_F_ON_R_T:
                {
                    adc_ = ((adc_range_ *     R_T) / (temp->R_F + R_T));
                    break;
                }
                case TEMP_SCHEMA_R_T_ON_R_F:
                {
                    adc_ = ((adc_range_ * temp->R_F) / (temp->R_F + R_T));
                    break;
                }
                default:
                {
                    // error
                    break;
                }
            }

            uint32_t const adc = (uint32_t)(adc_);

            return adc;
        }
    }

    return 0;
}

void bist_temp( void )
{
    fprintf( stdout, "Starting Temperature BIST\n" );

    TEMP temp;

    temp.V   = 3.3f;
    temp.R_F = 4700.0f; // 303 design

    temp.adc_range = 4096;

    temp.method = TEMP_METHOD_STEINHART_HART_BETA_R;
    temp.schema = TEMP_SCHEMA_R_F_ON_R_T;  // 303 design

    // From util_ntc
    temp.parameters.SH.A = 0.000674f;
    temp.parameters.SH.B = 0.000291f;
    temp.parameters.SH.C = 0.000000f;

    temp.parameters.SH.Beta = 3437.864258f;
    temp.parameters.SH.r    = 0.098243f;
    // From spec
    temp.parameters.SH.T0 = CVT_CELSIUS_TO_KELVIN_F( 25.0f );
    temp.parameters.SH.R0 = 10000.0f;

    float Tmin;
    float Tmax;

    ntc_T_minmax( &Tmin, &Tmax );

    uint32_t const adc_granularity = UINT32_C(128);
    uint32_t const adc_mask        = (adc_granularity - 1);

    uint32_t adc_min = ntc_get_adc_from_T( &temp, Tmin );
    uint32_t adc_max = ntc_get_adc_from_T( &temp, Tmax );

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

    fprintf( stdout, "Ideal 303\n" );

    for ( uint32_t adc = adc_start; adc <= adc_end; adc = adc + adc_granularity )
    {
        float const R_T = ntc_get_R_T_from_adc( &temp, adc );
        float const T = ntc_get_T_from_R_T( R_T );

        fprintf( stdout, "    ADC: %03" PRIX32 " => %5.1f 'C\n", adc, T );
    }

    for ( float T = 0.0f; T <= 100.0f; T = T + 5.0f )
    {
        uint32_t const adc = ntc_get_adc_from_T( &temp, T );

        fprintf( stdout, "    %5.1f 'C => ADC: %03" PRIX32 "\n", T, adc );
    }

    for ( TEMPMethod m = TEMP_METHOD_STEINHART_HART_BETA_R; (m <= TEMP_METHOD_KTY84_130_LINEAR); ++m )
    {
        fprintf( stdout, "Method %d\n", m );

        temp.method = m;

        for ( uint32_t adc = adc_start; adc <= adc_end; adc = adc + adc_granularity )
        {
            float const K = temp_read( &temp, adc );
            float const T = CVT_KELVIN_TO_CELSIUS_F( K );

            fprintf( stdout, "    ADC: %03" PRIX32 " => %5.1f 'C\n", adc, T );
        }

        for ( float T = 0.0f; T <= 100.0f; T = T + 5.0f )
        {
            float const K = CVT_CELSIUS_TO_KELVIN_F( T );
            uint32_t const adc = temp_get_adc( &temp, K );

            fprintf( stdout, "    %5.1f 'C => ADC %03" PRIX32 "\n", T, adc );
        }
    }

    fprintf( stdout, "Ideal 405\n" );

    // 405 design
    temp.R_F = 10000;
    temp.schema = TEMP_SCHEMA_R_T_ON_R_F;

    for ( uint32_t adc = adc_start; adc <= adc_end; adc = adc + adc_granularity )
    {
        float const R_T = ntc_get_R_T_from_adc( &temp, adc );
        float const T = ntc_get_T_from_R_T( R_T );

        fprintf( stdout, "    ADC: %03" PRIX32 " => %5.1f 'C\n", adc, T );
    }

    for ( float T = 0.0f; T <= 100.0f; T = T + 5.0f )
    {
        uint32_t const adc = ntc_get_adc_from_T( &temp, T );

        fprintf( stdout, "    %5.1f 'C => ADC: %03" PRIX32 "\n", T, adc );
    }

    for ( TEMPMethod m = TEMP_METHOD_STEINHART_HART_BETA_R; (m <= TEMP_METHOD_KTY84_130_LINEAR); ++m )
    {
        fprintf( stdout, "Method %d\n", m );

        temp.method = m;

        for ( uint32_t adc = adc_start; adc <= adc_end; adc = adc + adc_granularity )
        {
            float const K = temp_read( &temp, adc );
            float const T = CVT_KELVIN_TO_CELSIUS_F( K );

            fprintf( stdout, "    ADC: %03" PRIX32 " => %5.1f 'C\n", adc, T );
        }

        for ( float T = 0.0f; T <= 100.0f; T = T + 5.0f )
        {
            float const K = CVT_CELSIUS_TO_KELVIN_F( T );
            uint32_t const adc = temp_get_adc( &temp, K );

            fprintf( stdout, "    %5.1f 'C => ADC %03" PRIX32 "\n", T, adc );
        }
    }

    fprintf( stdout, "Finished Temperature BIST\n" );
}
