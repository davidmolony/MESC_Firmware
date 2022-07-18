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

const MOTOR_PROFILE_SIGNATURE = 'MMPE';

const MOTOR_PROFILE_SIZE = 36;

// MOTORProfile
function dump_MOTORProfile( profile )
{
    console.log( "dump_MOTORProfile" );
    var hex = '';

    hex = hex + dump_c_float(    profile.Imax         );
    hex = hex + dump_c_float(    profile.Vmax         );
    hex = hex + dump_c_float(    profile.Pmax         );
    hex = hex + dump_c_uint32_t( profile.RPMmax       );
    hex = hex + dump_c_uint8_t(  profile.pole_pairs   );
    hex = hex + dump_c_uint8_t(  profile.direction    );
    hex = hex + dump_c_uint8_t( 0 );
    hex = hex + dump_c_uint8_t( 0 );
    hex = hex + dump_c_float(    profile.Z_D          );
    hex = hex + dump_c_float(    profile.Z_Q          );
    hex = hex + dump_c_float(    profile.R            );
    hex = hex + dump_c_float(    profile.flux_linkage );
    hex = hex + dump_c_float(    profile.flux_linkage_min );
    hex = hex + dump_c_float(    profile.flux_linkage_max );
    hex = hex + dump_c_float(    profile.flux_linkage_gain );
    hex = hex + dump_c_float(    profile.non_linear_centering_gain );

    console.assert( hex.length == (NYBBLES_PER_BYTE * MOTOR_PROFILE_SIZE) );

    return hex;
}

function MOTORProfile()
{
    this.Imax = undefined;
    this.Vmax = undefined;
    this.Pmax = undefined;
    this.RPMmax = undefined;
    this.pole_pairs = undefined;
    this.direction = undefined;
    this.Z_D = undefined;
    this.Z_Q = undefined;
    this.R = undefined;
    this.flux_linkage = undefined;
    this.flux_linkage_min = undefined;
    this.flux_linkage_max = undefined;
    this.flux_linkage_gain = undefined;
    this.non_linear_centering_gain = undefined;
}

MOTORProfile.prototype.size = function()
{
    return MOTOR_PROFILE_SIZE;
}

MOTORProfile.prototype.dump = function()
{
    return dump_MOTORProfile( this );
}
