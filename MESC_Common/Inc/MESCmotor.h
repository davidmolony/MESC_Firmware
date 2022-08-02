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

#ifndef MESC_MOTOR_H
#define MESC_MOTOR_H

#include <inttypes.h>

#define MOTOR_PROFILE_SIGNATURE MAKE_UINT32_STRING('M','M','P','E')

#define NUM_HALL_STATES (UINT32_C(6))

struct HallEntry
{
    uint16_t min;
    uint16_t max;
    uint16_t mid;
    uint16_t width;
};

typedef struct HallEntry HallEntry;

struct MOTORProfile
{
    float       Imax;         // Amp
    float       Vmax;         // Volt
    float       Pmax;         // Watt
    uint32_t    RPMmax;       // 1/seconds
    uint8_t     pole_pairs;
    uint8_t     direction;
    uint8_t     _[2];
    float       L_D;          // Henry
    float       L_Q;          // Henry
    float       R;            // Ohm
    float       flux_linkage; // Weber
    float       flux_linkage_min;
    float       flux_linkage_max;
    float       flux_linkage_gain;
    float       non_linear_centering_gain;

    struct
    {
    uint16_t    encoder_offset;
    HallEntry   hall_states[NUM_HALL_STATES];
    }           sensor;
};

typedef struct MOTORProfile MOTORProfile;

extern MOTORProfile * motor_profile;

void motor_init( MOTORProfile const * const profile );

#endif
