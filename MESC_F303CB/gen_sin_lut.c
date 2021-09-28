
#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "sin_lut.h"

int main()
{
    const uint8_t MAX = ~((uint8_t )(0));
    const double  MID = (double)(MAX >> 1);

    for (uint32_t i = 0; i < SINLUT_ENTRIES; i = i + 1)
    {
        const double i_ = i;

        const double t = MID * sin( (2.0 * M_PI * i_) / SINLUT_ENTRIES );
        const uint8_t t_ = (uint8_t)(MID + t);
        if ((i % 16) == 0)
        {
            fprintf( stdout, "/*%" PRIX32 "x*/  ", (i >> 4) );
        }
        fprintf( stdout, "%*" PRIu8 ",%s", 3/*ceil(log10(MAX)*/, t_, (((i % 16) == 15) ? "\n" : "") );
    }

    return 0;
}
