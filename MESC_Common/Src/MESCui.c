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

#include "MESCui.h"

#include "MESCcli.h"
#include "MESCprofile.h"

#define UI_PROFILE_MAX_ENTRIES UINT32_C(8)

static UIProfile    ui_profile[UI_PROFILE_MAX_ENTRIES];
static char const * ui_profile_name[UI_PROFILE_MAX_ENTRIES];

void ui_init( UIProfile const * const profile )
{
    if (profile == PROFILE_DEFAULT)
    {
        uint32_t i = 0;
        uint32_t d = 0;
        uint32_t ui_length = sizeof(UIProfile);

        do
        {
            ProfileStatus const ret = profile_scan_entry(
                &i, UI_PROFILE_SIGNATURE,
                &ui_profile[d], &ui_length,
                &ui_profile_name[d] );

            if (ret != PROFILE_STATUS_SUCCESS)
            {
                return;
            }

            cli_reply( "UI ADD %d" "\r" "\n", ui_profile[d].type );

            ui_init( &ui_profile[d] );
            d++;
        }
        while (d < UI_PROFILE_MAX_ENTRIES);
    }

    switch (profile->type)
    {
    // Inputs
        case UI_PROFILE_THROTTLE:
            //profile->desc.throttle;
            break;
        case UI_PROFILE_BRAKE:
            //profile->desc.brake;
            break;
        case UI_PROFILE_BUTTON:
            //profile->desc.button;
            break;
    // Outputs
        case UI_PROFILE_INDICATOR:
            //profile->desc.indicator;
            break;
        case UI_PROFILE_SCREEN:
            //profile->desc.screen;
            break;
        default:
            break;
    }
}
