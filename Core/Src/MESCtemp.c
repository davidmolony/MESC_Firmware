
#include "MESCtemp.h"

#include <math.h>

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

static uint32_t temp_adc_range = UINT32_C(4096);    // Profile

/*
Conversions
*/

#define TEMP_0C_K (273.15f)
#define TEMP_CVT_C_TO_K(C) ((C) + TEMP_0C_K)
#define TEMP_CVT_K_TO_C(K) ((K) - TEMP_0C_K)
#if 0 // unused
static float temp_cvt_C_to_K( float const C )
{
    return TEMP_CVT_C_TO_K(C);
}

static float temp_cvt_K_to_C( float const K )
{
    return TEMP_CVT_K_TO_C(K);
}
#endif
/*
Approximation

x(t) = t + T_lo
y(t) = ln(R) = A * x(t) + B

ln(R) - B
--------- = x(t)
     A

    ln(R) - B
T = --------- - T_lo - X, let D = T_lo + X where X is fudge factor
        A
*/

static float const temp_approx_T_lo = -23.0f;           // Profile
static float const temp_approx_T_A  = - 0.034479692f;   // Profile
static float const temp_approx_T_B  =  11.0331166974f;  // Profile
static float const temp_approx_T_X  = 0.0f;//-26.51939842f;    // Profile
/*
Experimental (BIST)

ADC R_T     T   Actual (spec)
1D0   600   111  120
430  1666    82   80
770  4100    56   50
8D0  5800    46   40
AE0 10000    30   25
DC0 28722   - 1    0

Removing T_X appears to be more correct!
*/

static float temp_calculate_approximation( float const R_T )
{
    float const ln_R_T = log( R_T );
    float const T_D = (temp_approx_T_lo + temp_approx_T_X);
    float const T = ((ln_R_T - temp_approx_T_B) / temp_approx_T_A) + T_D;

    return T;
}
#if 0 // unused
/*
Steinhart & Hart method
*/

static float temp_SteinhartHart_A = 0.0f; // Profile
static float temp_SteinhartHart_B = 0.0f; // Profile
static float temp_SteinhartHart_C = 0.0f; // Profile

static void temp_calculate_SteinhartHart_ABC( float const (* const R)[3], float const (* const T)[3] )
{
    float L[3];
    float Y[3];
    float g[3];

    for ( int i = 0; i < 3; i++ )
    {
        L[i] = log( (*R)[i] );
        Y[i] = (1.0f / (*T)[i]);
    }

    g[1] = (Y[1] - Y[0]) / (L[1] - L[0]);
    g[2] = (Y[2] - Y[0]) / (L[2] - L[0]);

    float const L0_2 = (L[0] * L[0]);

    temp_SteinhartHart_C = ((g[2] - g[1]) / (L[2] - L[1])) * (1.0 / (L[0] + L[1] + L[2]));
    temp_SteinhartHart_B = (g[1] - (temp_SteinhartHart_C * (L0_2 + (L[0] * L[1]) + (L[1] * L[1]))));
    temp_SteinhartHart_A = (Y[0] - (L[0] * (temp_SteinhartHart_B + (temp_SteinhartHart_C * L0_2))));
}

static float temp_calculate_SteinhartHart( float const R_T )
{
    float const ln_R_T   = log( R_T );
    float const ln_R_T_3 = (ln_R_T * ln_R_T * ln_R_T);

    float const num = 1.0f;
    float const den = (temp_SteinhartHart_A + (temp_SteinhartHart_B * ln_R_T) + (temp_SteinhartHart_C * ln_R_T_3));
    float const T   = (num / den);

    return T;
}
#endif
/*
API
*/
//#include <stdio.h>//debug
float temp_read( uint32_t const adc_raw )
{
    float const adc  = adc_raw;
    float const Vout = ((TEMP_V * adc) / temp_adc_range);
    float const R_T = temp_calculate_R_T( Vout );

//fprintf( stderr, "R_T %.0f Ohm\n", R_T );//debug

    float const T = temp_calculate_approximation( R_T );
    //float const T = temp_calculate_SteinhartHart( R_T );

    return T;
}

uint32_t temp_get_adc( float const T )
{
    float const R_T = exp( ((T - temp_approx_T_lo) * temp_approx_T_A) + temp_approx_T_B );
    float const Vout = (TEMP_V * R_T) / (TEMP_R_F + R_T);
    uint32_t const adc_raw = (uint32_t)((Vout * ((float)temp_adc_range)) / TEMP_V);

    return adc_raw;
}
