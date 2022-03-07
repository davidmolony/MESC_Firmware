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

const TEMP_PROFILE_SIGNATURE = 'MTPE';

// TEMPMethod
const TEMP_METHOD_STEINHART_HART_BETA_R = 0;
//end

// TEMPSchema
const TEMP_SCHEMA_R_F_ON_R_T = 0;
const TEMP_SCHEMA_R_T_ON_R_F = 1;
//end

const TEMP_PROFILE_SIZE = 60;

// TEMPProfile
function dump_TEMPProfile( profile )
{
    console.log( "dump_TEMPProfile" );
    var hex = '';

    hex = hex + dump_c_int( profile.reading );
    hex = hex + dump_c_float( profile.V );
    hex = hex + dump_c_float( profile.R_F );
    
    hex = hex + dump_c_uint32_t( profile.adc_range );

    hex = hex + dump_c_int( profile.method );
    hex = hex + dump_c_int( profile.schema );

    switch (profile.method)
    {
        case TEMP_METHOD_STEINHART_HART_BETA_R:
            hex = hex + dump_c_float( profile.parameters_SH_A    );
            hex = hex + dump_c_float( profile.parameters_SH_B    );
            hex = hex + dump_c_float( profile.parameters_SH_C    );

            hex = hex + dump_c_float( profile.parameters_SH_Beta );
            hex = hex + dump_c_float( profile.parameters_SH_r    );

            hex = hex + dump_c_float( profile.parameters_SH_T0   );
            hex = hex + dump_c_float( profile.parameters_SH_R0   );

            hex = hex + dump_c_float( profile.limit_Tmin );
            hex = hex + dump_c_float( profile.limit_Tmax );
            break;
    }

    console.assert( hex.length == (NYBBLES_PER_BYTE * TEMP_PROFILE_SIZE) );

    return hex;
}

function TEMPProfile()
{
    this.reading = undefined;
    this.V = undefined;
    this.R_F = undefined;
    
    this.adc_range = undefined;

    this.method = undefined;
    this.schema = undefined;
    
    this.parameters_SH_A = undefined;
    this.parameters_SH_B = undefined;
    this.parameters_SH_C = undefined;

    this.parameters_SH_Beta = undefined;
    this.parameters_SH_r = undefined;

    this.parameters_SH_T0 = undefined;
    this.parameters_SH_R0 = undefined;

    this.limit_Tmin = undefined;
    this.limit_Tmax = undefined;
}

TEMPProfile.prototype.size = function()
{
    return TEMP_PROFILE_SIZE;
}

TEMPProfile.prototype.dump = function()
{
    return dump_TEMPProfile( this );
}

function TEMP_derive_SteinhartHart_ABC_from_Beta( profile )
{
    profile.parameters_SH_C = 0.0; // C is always zero when using Beta
    profile.parameters_SH_B = (1.0 / profile.parameters_SH_Beta);
    profile.parameters_SH_A = (profile.parameters_SH_T0 - (profile.parameters_SH_B * Math.log( profile.parameters_SH_R0 )));
}
