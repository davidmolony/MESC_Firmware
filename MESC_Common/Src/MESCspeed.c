/*
* Copyright 2021-2023 cod3b453
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

#include "MESCspeed.h"

#include "MESCmotor.h"
#include "MESCprofile.h"
#include "MESCcli.h"

#include "conversions.h"

#include <stdint.h>

SPEEDProfile const * speed_profile = NULL;

static float rev_speed; // Speedometer units per motor revolution

static float   const * speed_eHz  = NULL; // Motor revolutions
static uint8_t const * speed_pp   = NULL; // pole pairs

void speed_init( SPEEDProfile const * const profile )
{
    if (profile == PROFILE_DEFAULT)
    {
        static SPEEDProfile speed_profile_default =
		{
        	{0,{{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}}},
			{1,1},
			{26.0f,CONST_INCHES_PER_MILE_F}
		};
        uint32_t            speed_length = sizeof(speed_profile_default);

        ProfileStatus const ret = profile_get_entry(
            "SPD", SPEED_PROFILE_SIGNATURE,
            &speed_profile_default, &speed_length );

        speed_profile = &speed_profile_default;

        if (ret != PROFILE_STATUS_SUCCESS)
        {
            cli_reply( "SPD FAILED" "\r" "\n" );
        }
    }
    else
    {
    	speed_profile = profile;
    }

    float const gear_ratio          = (float)speed_profile->gear_ratio.motor
                                    / (float)speed_profile->gear_ratio.wheel;
    float const wheel_circumference = (speed_profile->wheel.diameter * CONST_PI_F);

    rev_speed = (gear_ratio * wheel_circumference * CONST_SECONDS_PER_HOUR_F)
              / speed_profile->wheel.conversion;
}

void speed_register_vars( float const * const eHz, uint8_t const * const pp )
{
    speed_eHz = eHz;
    speed_pp  = pp;
}

float speed_get( void )
{
    return ((*speed_eHz * rev_speed) / *speed_pp);
}
