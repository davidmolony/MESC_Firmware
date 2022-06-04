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

#ifndef MESC_UI_H
#define MESC_UI_H

#include <stdint.h>

#define UI_PROFILE_SIGNATURE MAKE_UINT32_STRING('M','U','P','E')

enum UIProfileType
{
    // Inputs
    UI_PROFILE_THROTTLE,
    UI_PROFILE_BRAKE,
    UI_PROFILE_BUTTON,

    // Outputs
    UI_PROFILE_INDICATOR,
    UI_PROFILE_SCREEN,
};

typedef enum UIProfileType UIProfileType;

enum UIResponse
{
    UI_RESPONSE_LINEAR,
    UI_RESPONSE_LOG,
};

typedef enum UIResponse UIResponse;

enum UIActivation
{
    UI_ACTIVATION_LEVEL,
    UI_ACTIVATION_EDGE,
};

typedef enum UIActivation UIActivation;

struct UIProfile
{
    UIProfileType   type;

    union
    {
    // Inputs
    struct
    {
    uint32_t        adc_min;
    uint32_t        adc_max;
    UIResponse      response;
    uint32_t        adc_trig;
    float           Imax;
    uint32_t        rcpwm_t_min;
    uint32_t        rcpwm_t_max;
    }               throttle;

    struct
    {
    uint32_t        adc_min;
    uint32_t        adc_max;
    UIResponse      response;
    uint32_t        adc_trig;
    float           Imax;
    uint32_t        _[2];
    }               brake;

    struct
    {
    uint32_t        interface;
    uint32_t        address;
    uint32_t        identifier;
    uint32_t        _[4];
    }               button;

    // Outputs
    struct
    {
    uint32_t        interface;
    uint32_t        address;
    uint32_t        identifier;
    UIActivation    activation;
    uint32_t        _[3];
    }               indicator;

    struct
    {
    uint32_t        interface;
    uint32_t        address;
    uint32_t        width;
    uint32_t        height;
    uint32_t        _[3];
    }               screen;

    }               desc;
};

typedef struct UIProfile UIProfile;

void ui_init( UIProfile const * const profile );

#endif
