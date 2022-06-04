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

#ifndef MESC_FINGERPRINT_H
#define MESC_FINGERPRINT_H

#include "string_op.h"

#define MESC_TIMESTAMP_YEAR   MAKE_UINT32_STRING('2','0','2','2')
#define MESC_TIMESTAMP_MONTH  MAKE_UINT16_STRING('0','2')
#define MESC_TIMESTAMP_DAY    MAKE_UINT16_STRING('0','6')
#define MESC_TIMESTAMP_HOUR   MAKE_UINT16_STRING('1','4')
#define MESC_TIMESTAMP_MINUTE MAKE_UINT16_STRING('4','6')
#define MESC_GITHASH_WORDS (160 / 32)
#define MESC_GITHASH {UINT32_C(0xc5f8904f),UINT32_C(0x0593d19b),UINT32_C(0x67d05c7b),UINT32_C(0xdd130ad3),UINT32_C(0x837addf0)}

struct MESCFingerprint
{
    uint32_t    year;              // Timestamp (MESC_TIMESTAMP_YEAR)
    uint16_t    month;             // Timestamp (MESC_TIMESTAMP_MONTH)
    uint16_t    day;               // Timestamp (MESC_TIMESTAMP_DAY)

    uint16_t    hour;              // Timestamp (MESC_TIMESTAMP_HOUR)
    uint16_t    minute;            // Timestamp (MESC_TIMESTAMP_MINUTE)

    uint8_t     _zero;             // Must be zero
    uint8_t     reserved[3];

    uint32_t    githash[MESC_GITHASH_WORDS]; // Git hash of firmware (MESC_GITHASH)
};

typedef struct MESCFingerprint MESCFingerprint;

#define MESC_FINGERPRINT {MESC_TIMESTAMP_YEAR,MESC_TIMESTAMP_MONTH,MESC_TIMESTAMP_DAY,MESC_TIMESTAMP_HOUR,MESC_TIMESTAMP_MINUTE,0,{0,0,0},MESC_GITHASH}

#endif
