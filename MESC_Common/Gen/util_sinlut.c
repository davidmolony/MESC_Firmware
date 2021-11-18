/*
* Copyright 2020-2021 cod3b453
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

#include "MESCsin_lut.h"

#include "bit_op.h"

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
    uint8_t const MAX = (uint8_t)~(UINT8_C(0));
    double  const MID = (double)(MAX >> 1);

    for ( uint32_t i = 0; i < SINLUT_ENTRIES; i = i + 1 )
    {
        double const i_ = i;

        double  const t  = MID * sin( (2.0 * M_PI * i_) / SINLUT_ENTRIES );
        uint8_t const t_ = (uint8_t)(MID + t);

        if ((i % (UINT32_C(1) << BITS_PER_NYBBLE)) == 0)
        {
            fprintf( stdout, "/*%" PRIX32 "x*/  ", (i >> BITS_PER_NYBBLE) );
        }

        fprintf( stdout, "%*" PRIu8 ",%s", 3/*ceil(log10(MAX)*/, t_, (((i % (UINT32_C(1) << BITS_PER_NYBBLE)) == BIT_MASK_32(BITS_PER_NYBBLE)) ? "\n" : "") );
    }

    return EXIT_SUCCESS;
}
