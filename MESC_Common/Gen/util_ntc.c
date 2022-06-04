/*
* Copyright 2021-2022 cod3b453
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

#include "util_ntc.h"

#include "conversions.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static float const Steinhart_Hart_T0 = CVT_CELSIUS_TO_KELVIN_F( 25.0f ); // Profile (from NTC spec)
static float const Steinhart_Hart_R0 = 10000.0f; // Profile (from NTC spec)

static void derive_Steinhart_Hart( NTCNode const * node, size_t const count )
{
#define DEGREE (3)
#define PARTITIONS ((2 * DEGREE) + 1)
    NTCNode const * p[DEGREE];

    for ( size_t i = 0, j = 1; j < PARTITIONS; ++i, j = j + 2 )
    {
        size_t const k = ((count * j) / PARTITIONS);
        p[i] = &node[k];
    }

    float L[3];
    float Y[3];
    float sumL = 0.0f;

    for ( uint32_t i = 0; i < 3; ++i )
    {
        L[i] = logf( p[i]->R );
        sumL += L[i];
        float const K = CVT_CELSIUS_TO_KELVIN_F( p[i]->T );
        Y[i] = (1.0f / K);
    }

    float G[3];

    G[1] = ((Y[1] - Y[0])
         /  (L[1] - L[0]));

    G[2] = ((Y[2] - Y[0])
         /  (L[2] - L[0]));

    float const C = ((G[2] - G[1])
                  / ((L[2] - L[1]) * sumL));
    float const B = (G[1] - (C * ((L[0] * L[0]) + (L[1] * L[0]) + (L[1] * L[1]))));
    float const A = (Y[0] - (L[0] * (B + (C * L[0] * L[0]))));

    float const Beta = (1.0f / B);
    float const r = Steinhart_Hart_R0 * expf( -Beta / Steinhart_Hart_T0 );

    fprintf( stderr, "Code Generation\n"
                     "STARTING------------------------------------------------------------------------\n" );

    fprintf( stdout, "static float const Steinhart_Hart_A = %ff;\n"
                     "static float const Steinhart_Hart_B = %ff;\n"
                     "static float const Steinhart_Hart_C = %ff;\n"
                     "static float const Steinhart_Hart_Beta = %ff;\n"
                     "static float const Steinhart_Hart_r = %ff;\n",
                     A,
                     B,
                     C,
                     Beta,
                     r );

    fprintf( stderr, "FINISHED------------------------------------------------------------------------\n" );
#undef DEGREE
#undef PARTITIONS
}

int main ( int argc, char ** argv )
{
    derive_Steinhart_Hart( ntc_T_R, ntc_T_R_count );

    return EXIT_SUCCESS;
    (void)argc;
    (void)argv;
}
