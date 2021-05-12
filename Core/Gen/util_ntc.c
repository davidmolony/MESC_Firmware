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

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "util_ntc.h"

static float get_T_min( NTCNode const * node, size_t const count )
{
    float min = FLT_MAX;

    for ( size_t i = 0; i < count; ++i )
    {
        if (node[i].T < min)
        {
            min = node[i].T;
        }
    }
    assert( min != FLT_MAX );
    return min;
}

static NTCNode * replot( NTCNode const * node, size_t const count, float const Tmin )
{
    NTCNode * ret = (NTCNode *)calloc( count, sizeof(NTCNode) );
    assert(ret);

    fprintf( stderr, "===========================\n"
                     "  T  |   R   || Tadj |ln(R)\n"
                     "---------------------------\n" );

    for ( size_t i = 0; i < count; ++i )
    {
        float const Tadj = node[i].T - Tmin;
        float const ln_R = logf( node[i].R );

        ret[i].T = Tadj;
        ret[i].R = ln_R;

        fprintf( stderr, "%5.1f %6.0f -> %5.1f %5.2f\n",
            node[i].T, node[i].R,
             ret[i].T,  ret[i].R );
    }

    fprintf( stderr, "===========================\n" );

    return ret;
}

#define DEGREE (2)
#define PARTITIONS ((2 * DEGREE) + 1)

static void derive_trend( NTCNode const * node, size_t const count, float const Tmin )
{
    NTCNode p[DEGREE];

    for ( size_t i = 0, j = 1; j < PARTITIONS; ++i, j = j + 2 )
    {
        size_t const k = ((count * j) / PARTITIONS);
        p[i] = node[k];
    }

    float const g = (p[1].R - p[0].R) / (p[1].T - p[0].T);
    float const c = (p[0].R - (g * p[0].T));

    fprintf( stderr, "\n"
                     "Log Approximation\n"
                     "\n"
                     "    ln(R) = %f x (T - %f) + %f\n"
                     "\n",
                     g, Tmin, c );

    fprintf( stderr, "Inverse Approximation\n"
                     "\n"
                     "        ln(R) - %f\n"
                     "    T = ----------------- - %f\n"
                     "            %f\n"
                     "\n",
                     c,
                     Tmin,
                     g );

    fprintf( stderr, "Code Generation\n"
                     "STARTING------------------------------------------------------------------------\n" );

    fprintf( stdout, "static float const temp_approx_T_lo = %ff;\n"
                     "static float const temp_approx_T_A  = %ff;\n"
                     "static float const temp_approx_T_B  = %ff;\n",
                     Tmin,
                     g,
                     c );

    fprintf( stderr, "FINISHED------------------------------------------------------------------------\n" );
}

int main ( int argc, char ** argv )
{
    float const Tmin = get_T_min( ntc_T_R, ntc_T_R_count );

    NTCNode * tbl = replot( ntc_T_R, ntc_T_R_count, Tmin );

    derive_trend( tbl, ntc_T_R_count, Tmin );

    free( tbl );

    return EXIT_SUCCESS;
    (void)argc;
    (void)argv;
}
