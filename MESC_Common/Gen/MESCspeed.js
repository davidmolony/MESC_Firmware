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

const SPEED_PROFILE_SIGNATURE = 'MSPE';

const SPEED_PROFILE_SIZE = 44;

// SPEEDProfile
function dump_SPEEDProfile( profile )
{
    console.log( "dump_SPEEDProfile" );
    var hex = '';

    hex = hex + dump_c_uint16_t( profile.sensor_encoder_offset );
    hex = hex + dump_c_uint16_t( 0 );
    for ( let h = 0; h < 6; h++ )
    {
        hex = hex + dump_c_uint16_t( profile.sensor_hall_states[h].min );
        hex = hex + dump_c_uint16_t( profile.sensor_hall_states[h].max );
    }

    hex = hex + dump_c_uint32_t( profile.gear_ratio_motor );
    hex = hex + dump_c_uint32_t( profile.gear_ratio_wheel );

    hex = hex + dump_c_float( profile.wheel_diameter );
    hex = hex + dump_c_float( profile.wheel_conversion );

    console.assert( hex.length == (NYBBLES_PER_BYTE * SPEED_PROFILE_SIZE) );

    return hex;
}

function HallEntry()
{
    this.min = undefined;
    this.max = undefined;
}

function SPEEDProfile()
{
    this.sensor_encoder_offset = undefined;
    this.sensor_hall_states = new Array(6);
    for ( let h = 0; h < 6; h++ )
    {
        this.sensor_hall_states[h] = new HallEntry();
    }

    this.gear_ratio_motor = undefined;
    this.gear_ratio_wheel = undefined;

    this.wheel_diameter = undefined;
    this.wheel_conversion = undefined;
}

SPEEDProfile.prototype.size = function()
{
    return SPEED_PROFILE_SIZE;
}

SPEEDProfile.prototype.dump = function()
{
    return dump_SPEEDProfile( this );
}
