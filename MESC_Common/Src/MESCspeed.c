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

#include "MESCspeed.h"

#include "MESCcli.h"
#include "MESCprofile.h"

#include "conversions.h"

#include <stddef.h>

static SPEEDProfile road_profile;
static SPEEDProfile diag_profile;
static SPEEDProfile const * speed_profile = NULL;

static float rev_speed; // Speedometer units per motor revolution

static float const * speed_drev = NULL; // Motor revolutions (i.e. 0..1 is 0..2Pi radians)
static float const * speed_dt   = NULL; // time (seconds) // TODO

static void speed_profile_diag( void )
{
/*
NOTE

The diagnostic profile is for hardware bring-up and debugging purposes only
*/
    uint32_t            diag_length = sizeof(diag_profile);

    ProfileStatus const ret = profile_get_entry(
        "SPDD", SPEED_PROFILE_SIGNATURE,
        &diag_profile, &diag_length );

    if (ret != PROFILE_STATUS_SUCCESS)
    {
        cli_reply( "SPDD FAILED\r\n" );
        return;
    }

    speed_init( &diag_profile );
    cli_reply( "SPDD LOADED\r\n" );
}

static void speed_profile_road( void )
{
/*
NOTE

This should be the default profile when actually using the system and match
whatever local requirements there are.
*/
    uint32_t            road_length = sizeof(road_profile);

    ProfileStatus const ret = profile_get_entry(
        "SPDR", SPEED_PROFILE_SIGNATURE,
        &road_profile, &road_length );

    if (ret != PROFILE_STATUS_SUCCESS)
    {
        cli_reply( "SPDR FAILED\r\n" );
        return;
    }

    speed_init( &road_profile );
    cli_reply( "SPDR LOADED\r\n" );
}

static void speed_profile_toggle( void )
{
    if (speed_profile == NULL)
    {
        return;
    }

    if (speed_profile == &road_profile)
    {
        speed_profile_diag();
    }
    else if (speed_profile == &diag_profile)
    {
        speed_profile_road();
    }
}

void speed_init( SPEEDProfile const * const profile )
{
    // If initialising...
    if (profile == PROFILE_DEFAULT)
    {
        // ...attempt to to load the road profile...
        speed_profile_road();
        // DANGER - This is re-entrant

        // ...otherwise fall back to diagnostic profile
        if (speed_profile == NULL)
        {
			speed_profile_diag();
			// DANGER - This is re-entrant

			if (speed_profile == NULL)
			{
				speed_profile = &road_profile;
			}
        }
    }
    else
    {
    	speed_profile = profile;
    }

    cli_register_function( "spd_diag"  , speed_profile_diag   );
    cli_register_function( "spd_road"  , speed_profile_road   );
    cli_register_function( "spd_toggle", speed_profile_toggle );

    float const gear_ratio          = (float)speed_profile->gear_ratio.motor
                                    / (float)speed_profile->gear_ratio.wheel;
    float const wheel_circumference = (speed_profile->wheel.diameter * CONST_PI_F);

    rev_speed = (gear_ratio * wheel_circumference * CONST_SECONDS_PER_HOUR_F)
              / speed_profile->wheel.conversion;
}

void speed_register_vars( float const * const drev, float const * const dt )
{
    speed_drev = drev;
    speed_dt   = dt;
}

float speed_get( void )
{
    return ((*speed_drev * rev_speed) / *speed_dt);
}
