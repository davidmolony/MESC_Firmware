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

const TEMP_PROFILE_SIGNATURE = 'MTPE';

// TEMPMethod
const TEMP_METHOD_CURVE_APPROX          = 0;

const TEMP_METHOD_STEINHART_HART_ABC    = 1;
const TEMP_METHOD_STEINHART_HART_BETA_R = 2;
//end

// TEMPSchema
const TEMP_SCHEMA_R_F_ON_R_T = 0;
const TEMP_SCHEMA_R_T_ON_R_F = 1;
//end

// TEMPProfile
function dump_TEMPProfile( profile )
{
    var hex = '';

    hex = hex + dump_c_float( profile.V );
    hex = hex + dump_c_float( profile.R_F );

    hex = hex + dump_c_uint32_t( profile.adc_range );

    hex = hex + dump_c_int( profile.method );
    hex = hex + dump_c_int( profile.schema );

    // TEMP_METHOD_CURVE_APPROX
    hex = hex + dump_c_float( profile.parameters.approx.A   );
    hex = hex + dump_c_float( profile.parameters.approx.B   );
    //hex = hex + dump_c_char( '', 4 ); //union

    //hex = hex + dump_c_char( '', 4 ); //union
    //hex = hex + dump_c_char( '', 4 ); //union

    hex = hex + dump_c_float( profile.parameters.approx.Tlo );
    //hex = hex + dump_c_char( '', 4 ); //union

    // TEMP_METHOD_STEINHART_HART_ABC
    // TEMP_METHOD_STEINHART_HART_BETA_R
    hex = hex + dump_c_float( profile.parameters.SH.A    );
    hex = hex + dump_c_float( profile.parameters.SH.B    );
    hex = hex + dump_c_float( profile.parameters.SH.C    );

    hex = hex + dump_c_float( profile.parameters.SH.Beta );
    hex = hex + dump_c_float( profile.parameters.SH.r    );

    hex = hex + dump_c_float( profile.parameters.SH.T0   );
    hex = hex + dump_c_float( profile.parameters.SH.R0   );

    hex = hex + dump_c_float( profile.limit.Tmin );
    hex = hex + dump_c_float( profile.limit.Tmax );

    return hex;
}
