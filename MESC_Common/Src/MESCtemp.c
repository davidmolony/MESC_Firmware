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

#include "stm32fxxx_hal.h"

#include "conversions.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/*
Schematic

TEMP_SCHEMA_R_F_ON_R_T

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

TEMP_SCHEMA_R_T_ON_R_F

     -+- V
     \|
     |\| R_T
     |_\_
      |
      +- Vout - >ADC
      |
     | | R_F = 10k
     |_|
      |
     -+-

          V * R_F
    R_T = ------- - R_F
            Vout
*/

static float temp_calculate_R_T( TEMP const * const temp, float const Vout )
{
    if (temp == NULL)
    {
        return 0.0f;
    }

    switch (temp->schema)
    {
        case TEMP_SCHEMA_R_F_ON_R_T:
        {
            float const num = (Vout * temp->R_F);
            float const den = (temp->V - Vout);
            float const R_T = (num / den);
            // return elec_potdiv_Rlo( temp->V, Vout, temp->R_F );
            return R_T;
        }
        case TEMP_SCHEMA_R_T_ON_R_F:
        {
            float const num = (temp->V * temp->R_F);
            float const den = Vout;
            float const R_T = (num / den) - temp->R_F;
            // return elec_potdiv_Rhi( temp->V, Vout, temp->R_F );
            return R_T;
        }
        default:
        {
            // error
            return 0.0f;
        }
    }
}
#if 0 // STEINHART_HART_ABC
/*
Steinhart & Hart A/B/C method
*/

static void temp_derive_SteinhartHart_ABC_from_points( TEMPProfile * const profile, float const (* const R)[3], float const (* const T)[3] )
{
    float L[3];
    float Y[3];
    float g[3];

    for ( int i = 0; i < 3; i++ )
    {
        L[i] = logf( (*R)[i] );
        Y[i] = (1.0f / (*T)[i]);
    }

    g[1] = (Y[1] - Y[0]) / (L[1] - L[0]);
    g[2] = (Y[2] - Y[0]) / (L[2] - L[0]);

    float const L0_2 = (L[0] * L[0]);

    profile->parameters.SH.C = ((g[2] - g[1]) / (L[2] - L[1])) * (1.f / (L[0] + L[1] + L[2]));
    profile->parameters.SH.B = (g[1] - (profile->parameters.SH.C * (L0_2 + (L[0] * L[1]) + (L[1] * L[1]))));
    profile->parameters.SH.A = (Y[0] - (L[0] * (profile->parameters.SH.B + (profile->parameters.SH.C * L0_2))));
}

static void temp_derive_SteinhartHart_Beta_r_from_ABC( TEMPProfile * profile )
{
    profile->parameters.SH.Beta = (1.0f / profile->parameters.SH.B);
    profile->parameters.SH.r    = profile->parameters.SH.R0 * expf( -profile->parameters.SH.Beta / profile->parameters.SH.T0 );
}

static float temp_calculate_SteinhartHart_ABC( float const R_T )
{
    assert(temp_profile != NULL);
    float const ln_R_T   = logf( R_T );
    float const ln_R_T_3 = (ln_R_T * ln_R_T * ln_R_T);

    float const num = 1.0f;
    float const den = (temp_profile->parameters.SH.A + (temp_profile->parameters.SH.B * ln_R_T) + (temp_profile->parameters.SH.C * ln_R_T_3));
    float const T   = (num / den);

    return T;
}
#endif
/*
Steinhart & Hart Beta/r method
*/
#if 0 // STEINHART_HART_ABC
static void temp_derive_SteinhartHart_ABC_from_Beta( TEMPProfile * const profile )
{
    profile->parameters.SH.C = 0.0f; // C is always zero when using Beta
    profile->parameters.SH.B = (1.0f / profile->parameters.SH.Beta);
    profile->parameters.SH.A = (profile->parameters.SH.T0 - (profile->parameters.SH.B * logf( profile->parameters.SH.R0 )));
}
#endif
static float temp_calculate_SteinhartHart_Beta_r( TEMP const * const temp, float const R_T )
{
    return temp->parameters.SH.Beta / logf( R_T / temp->parameters.SH.r );
}

static float temp_calculate_KTY83_122_Linear( float const R_T )
{
    return 0.10168f*R_T + 202.0f; //Function linearised from 10-160degC with <6degC max error, 3.8degC in mid range.
}

static float temp_calculate_KTY84_130_Linear( float const R_T )
{
    return 0.14879f*R_T + 216.0f; //Function linearised from 10-240degC with <10degC max error.
}

/*
API
*/
float temp_read( TEMP const * const temp, uint32_t const adc_raw )
{
    float const adc  = (float)adc_raw;
    float const Vout = ((temp->V * adc) / temp->adc_range);
    float const R_T = temp_calculate_R_T( temp, Vout );

    float T;

    switch (temp->method)
    {
        case TEMP_METHOD_STEINHART_HART_BETA_R:
        {
            T = temp_calculate_SteinhartHart_Beta_r( temp, R_T );
            break;
        }
        case TEMP_METHOD_KTY83_122_LINEAR:
        {
            T = temp_calculate_KTY83_122_Linear( R_T );
            break;
        }
        case TEMP_METHOD_KTY84_130_LINEAR:
        {
            T = temp_calculate_KTY84_130_Linear( R_T );
            break;
        }
        default:
        {
            T = 0.0f;
            break;
        }
    }

    return T;
}

uint32_t temp_get_adc( TEMP const * const temp, float const T )
{
    if (temp == NULL)
    {
        return 0;
    }

    float R_T;

    switch (temp->method)
    {
        case TEMP_METHOD_STEINHART_HART_BETA_R:
        {
            R_T = temp->parameters.SH.r * expf( temp->parameters.SH.Beta / T );
            // OR R_T =  R0 * exp( Beta * (1 / T - 1 / T0) )
            break;
        }
        case TEMP_METHOD_KTY83_122_LINEAR:
        {
            R_T = (T - 202.0f) / 0.10168f;
            break;
        }
        case TEMP_METHOD_KTY84_130_LINEAR:
        {
            R_T = (T - 216.0f) / 0.14879f;
            break;
        }
        default:
        {
            R_T = 0.0f;
            break;
        }
    }

    float Vout;

    switch (temp->schema)
    {
        case TEMP_SCHEMA_R_F_ON_R_T:
        {
            Vout = (temp->V *       R_T) / (temp->R_F + R_T);
            //Vout = elec_potdiv_Vout( temp_profile->V, temp_profile->R_F, R_T );
            break;
        }
        case TEMP_SCHEMA_R_T_ON_R_F:
        {
            Vout = (temp->V * temp->R_F) / (temp->R_F + R_T);
            //Vout = elec_potdiv_Vout( temp_profile->V, R_T, temp_profile->R_F );
            break;
        }
        default:
        {
            Vout = 0.0f;
            break;
        }
    }

    uint32_t const adc_raw = (uint32_t)((Vout * ((float)temp->adc_range)) / temp->V);

    return adc_raw;
}

TEMPState temp_check( TEMP const * const temp, float const T, float * const dT )
{
	// If there is no temperature reading, assume it is OK
    if (temp == NULL)
    {
        return TEMP_STATE_OK;
    }
	// If the temperature is (suspiciously) too cold, assume it is OK
	if (T <= temp->limit.Tmin)
	{
		return TEMP_STATE_OK;
	}
	// If the temperature is below hot, it is fine
	else if (T <= temp->limit.Thot)
	{
		return TEMP_STATE_OK;
	}
	// If the temperature is hot but below the maximum return the temperature overshoot for correction
	else if (T < temp->limit.Tmax)
	{
		if (dT != NULL)
		{
			*dT = T - temp->limit.Thot;
		}
		return TEMP_STATE_ROLLBACK;
	}
	// Otherwise it has overheated
	if (dT != NULL)
	{
		*dT = T - temp->limit.Thot;
	}
	return TEMP_STATE_OVERHEATED;
}

TEMPState temp_check_raw( TEMP const * const temp, uint32_t const adc_raw, float * const dT )
{
	// If there is no temperature reading, assume it is OK
    if (temp == NULL)
    {
        return TEMP_STATE_OK;
    }

    float const T = temp_read( temp, adc_raw );
    return temp_check( temp, T, dT );
}
