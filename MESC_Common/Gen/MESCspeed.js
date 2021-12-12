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

const SPEED_PROFILE_SIGNATURE = 'MSPE';

// SPEEDProfile
function dump_SPEEDProfile( profile )
{
    var hex = '';

    hex = hex + dump_c_float( profile.motor.Imax );
    hex = hex + dump_c_float( profile.motor.Vmax );
    hex = hex + dump_c_float( profile.motor.Pmax );
    hex = hex + dump_c_uint32_t( profile.motor.RPMmax );
    hex = hex + dump_c_uint8_t( profile.motor.pole_pairs );
    hex = hex + dump_c_uint8_t( profile.motor.direction );
    hex = hex + dump_c_uint8_t( profile.motor.allow_regen );
    hex = hex + dump_c_uint8_t( 0 );

    hex = hex + dump_c_uint8_t( profile.sensor.type );
    for ( let h = 0; h < 6; h++ )
    {
        hex = hex + dump_c_uint8_t( profile.sensor.hall_states[h] );
    }
    hex = hex + dump_c_uint8_t( 0 );

    hex = hex + dump_c_uint32_t( profile.gear_ratio.motor );
    hex = hex + dump_c_uint32_t( profile.gear_ratio.wheel );

    hex = hex + dump_c_float( profile.wheel.diameter );
    hex = hex + dump_c_float( profile.wheel.conversion );

    return hex;
}
