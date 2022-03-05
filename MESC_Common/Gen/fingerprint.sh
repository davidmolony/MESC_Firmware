#!/bin/bash

# Copyright 2021-2022 cod3b453
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
OUTFIL=${SCRIPT_DIR}/../Inc/fingerprint.h

cat << HEAD > ${OUTFIL}
/*
* Copyright 2021-$(date "+%Y") cod3b453
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

#include <stdint.h>

HEAD

TIMESTAMP=$(date "+%Y%m%d%H%M")

1>&2 echo "INFO: TIMESTAMP is ${TIMESTAMP}"

echo "#define MESC_TIMESTAMP_YEAR   MAKE_UINT32_STRING('${TIMESTAMP:0:1}','${TIMESTAMP:1:1}','${TIMESTAMP:2:1}','${TIMESTAMP:3:1}')"  >> ${OUTFIL}
echo "#define MESC_TIMESTAMP_MONTH  MAKE_UINT16_STRING('${TIMESTAMP:4:1}','${TIMESTAMP:5:1}')"  >> ${OUTFIL}
echo "#define MESC_TIMESTAMP_DAY    MAKE_UINT16_STRING('${TIMESTAMP:6:1}','${TIMESTAMP:7:1}')"  >> ${OUTFIL}
echo "#define MESC_TIMESTAMP_HOUR   MAKE_UINT16_STRING('${TIMESTAMP:8:1}','${TIMESTAMP:9:1}')"  >> ${OUTFIL}
echo "#define MESC_TIMESTAMP_MINUTE MAKE_UINT16_STRING('${TIMESTAMP:10:1}','${TIMESTAMP:11:1}')" >> ${OUTFIL}

GITHASH=$(git log -n 1 | head -n 1 | awk '{print $2}')

1>&2 echo "INFO: GITHASH is ${GITHASH}"

echo "#define MESC_GITHASH_WORDS (160 / 32)" >> ${OUTFIL}
echo "#define MESC_GITHASH {UINT32_C(0x${GITHASH:32:8}),UINT32_C(0x${GITHASH:24:8}),UINT32_C(0x${GITHASH:16:8}),UINT32_C(0x${GITHASH:8:8}),UINT32_C(0x${GITHASH:0:8})}" >> ${OUTFIL}

cat << 'FOOT' >> ${OUTFIL}

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
FOOT

cat << HEADJS > ${SCRIPT_DIR}/fingerprint.js
/*
* Copyright 2021-$(date "+%Y") cod3b453
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

HEADJS

echo "var TIMESTAMP = '${TIMESTAMP}'" >> ${SCRIPT_DIR}/fingerprint.js
echo "var GITHASH = '${GITHASH}'" >> ${SCRIPT_DIR}/fingerprint.js

# Provided for BIST virtual FLASH fingerprint generation
echo "${TIMESTAMP}${GITHASH}"

