
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

static void derive_trend( NTCNode const * node, size_t const count, float const Tmin )
{
    static size_t const DEGREE = 2;
    static size_t const PARTITIONS = ((2 * DEGREE) + 1);

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
