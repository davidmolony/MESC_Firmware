
#include "MESCsin_lut.h"

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

int main()
{
    uint8_t const MAX = ~((uint8_t)(0));
    double  const MID = (double)(MAX >> 1);

    for ( uint32_t i = 0; i < SINLUT_ENTRIES; i = i + 1 )
    {
        double const i_ = i;

        double  const t  = MID * sin( (2.0 * M_PI * i_) / SINLUT_ENTRIES );
        uint8_t const t_ = (uint8_t)(MID + t);

        if ((i % (UINT32_C(1) << 4/*BITS_PER_NYBBLE*/)) == 0)
        {
            fprintf( stdout, "/*%" PRIX32 "x*/  ", (i >> 4/*BITS_PER_NYBBLE*/) );
        }

        fprintf( stdout, "%*" PRIu8 ",%s", 3/*ceil(log10(MAX)*/, t_, (((i % (UINT32_C(1) << 4/*BITS_PER_NYBBLE*/)) == 15) ? "\n" : "") );
    }

    return EXIT_SUCCESS;
}
