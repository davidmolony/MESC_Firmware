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

#include "conversions.h"

static float wheel_circumference; // (wheel units)
static float wheel_distance; // (wheel units) per (distance unit)
static float pole_pairs;
//static float factor = 6.0f;
static float speed; // (distance unit) per hour

static float gear_ratio; // gear_motor / gear_wheel

void speed_set_motor( uint32_t const pole_pairs_ )
{
    pole_pairs = (float)pole_pairs_;
}

void speed_set_wheel( float const diameter )
{
    wheel_circumference = CONST_PI_F * diameter;
}

void speed_set_units( float const wheel_distance_ )
{
    wheel_distance = wheel_distance_;
}

void speed_set_gear_ratio(
    uint32_t const gear_motor,
    uint32_t const gear_wheel )
{
    gear_ratio = ((float)gear_motor) / ((float)gear_wheel);
}

float speed_get( void )
{
    float const dtheta = 0.0f; // revolutions // TODO
    float const dS = (dtheta * gear_ratio * wheel_circumference) / wheel_distance; // distamce
    float const dt = 0.0f; // time (seconds) // TODO

    float const v = ((CONST_SECONDS_PER_HOUR_F * dS) / dt);

    speed = ((speed * 255.0f) + v) / 256.0f; // TODO

    return speed;
}
