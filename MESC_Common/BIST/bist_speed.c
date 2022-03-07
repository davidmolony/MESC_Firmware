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

#include "MESCspeed.h"

#include "conversions.h"

#include <math.h>
#include <stdio.h>

static float drev = 0.0f;
static float dt   = 0.0f;

void bist_speed( void )
{
    fprintf( stdout, "Starting Speed BIST\n" );

    speed_register_vars( &drev, &dt );

    SPEEDProfile sp;

    sp.motor.Imax         =  40.0f;
    sp.motor.Vmax         = 100.0f;
    sp.motor.Pmax         = 250.0f;

    sp.motor.RPMmax       = 1000;

    sp.motor.pole_pairs   = 6;
    sp.motor.direction    = 0;

    sp.motor.Z_D          = 0.0;
    sp.motor.Z_Q          = 0.0;
    sp.motor.R            = 0.0;
    sp.motor.flux_linkage = 0.0;

    sp.sensor.encoder_offset = 0;

    for ( uint32_t h = 0; h < NUM_HALL_STATES; h++ )
    {
        sp.sensor.hall_states[h].min = 0;
        sp.sensor.hall_states[h].max = 0;
    }

    sp.gear_ratio.motor   = 1;
    sp.gear_ratio.wheel   = 1;

    sp.wheel.diameter     = 26.0f; // Inches
    sp.wheel.conversion   = CONST_INCHES_PER_MILE_F;

    speed_init( &sp );

    // Extract internal variable using identity parameters
    drev = 1.0f;
    dt   = 1.0f;

    float const rev_speed = speed_get();

    // Sweep rpm range
    fprintf( stdout, "RPM sweep\n" );

    dt = CONST_SECONDS_PER_MINUTE_F;

    for ( uint32_t i = 1; i <= 10; ++i )
    {
        uint32_t rpm = (sp.motor.RPMmax * i) / 10;

        rpm = ((rpm / 100) * 100);

        drev = (float)rpm;

        float const S = speed_get();

        fprintf( stdout, "    %5" PRIu32 " rpm => %4.1f uph\n", rpm, S );
    }

    // Sweep speed range
    fprintf( stdout, "Speed sweep\n" );

    drev = 1.0f;

    for ( float v = 5.0f; v < 50.0f; v = v + 5.0f )
    {
        dt = (drev * rev_speed) / v;
        fprintf( stdout, "    %4.1f uph is %4.2f seconds per motor revolution (%5.0f rpm)\n", v, dt, (CONST_SECONDS_PER_MINUTE_F / dt) );
    }

    // Sweep rotation rate
    fprintf( stdout, "Rotation sweep\n" );

    for ( int32_t f = -5; f <= 10; )
    {
        if (f < 0)
        {
            dt = fabsf( (float)f );
        }
        else if (f == 0)
        {
            dt = 1.0f;
        }
        else
        {
            dt = 1.0f / fabsf( (float)f );
        }

        float const S = speed_get();

        fprintf( stdout, "    %4.2f seconds per motor revolution (%5.0f rpm) => %4.1f uph\n", dt, (CONST_SECONDS_PER_MINUTE_F / dt), S );

        if (f == -1)
        {
            f = 1;
        }

        f++;
    }

    fprintf( stdout, "Finished Speed BIST\n" );
}
