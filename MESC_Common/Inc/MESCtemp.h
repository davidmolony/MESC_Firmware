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

#ifndef MESC_TEMP_H
#define MESC_TEMP_H

#include <stdbool.h>
#include <stdint.h>

#define TEMP_PROFILE_SIGNATURE MAKE_UINT32_STRING('M','T','P','E')

enum TEMPMethod
{
    TEMP_METHOD_STEINHART_HART_BETA_R,
};

typedef enum TEMPMethod TEMPMethod;

enum TEMPSchema
{
    TEMP_SCHEMA_R_F_ON_R_T,
    TEMP_SCHEMA_R_T_ON_R_F
};

typedef enum TEMPSchema TEMPSchema;

enum TEMPReading
{
    TEMP_READING_BOARD,
    TEMP_READING_MOTOR,
};

typedef enum TEMPReading TEMPReading;

struct TEMPProfile
{
    TEMPReading reading;
    float       V;
    float       R_F;

    uint32_t    adc_range;

    TEMPMethod  method;
    TEMPSchema  schema;

    union
    {
    struct
    {
    float       A;
    float       B;
    float       C;

    float       Beta;
    float       r;

    float       T0;
    float       R0;
    }           SH;
    }           parameters;

    struct
    {
    float       Tmin;
    float       Tmax;
    }           limit;
};

typedef struct TEMPProfile TEMPProfile;

void temp_init( TEMPProfile const * const profile );

float temp_read( uint32_t const adc_raw );

uint32_t temp_get_adc( float const T );

bool temp_check( uint32_t const adc_raw );

#endif
