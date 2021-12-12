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

const BAT_PROFILE_SIGNATURE = 'MBPE';

// BATDisplay
const BAT_DISPLAY_PERCENT = 0;
const BAT_DISPLAY_AMPHOUR = 1;
//end

// BATProfile
function dump_BATProfile()
{
    var hex = '';

    hex = hex + dump_c_float( profile.cell.Imax );
    hex = hex + dump_c_float( profile.cell.Vmax );
    hex = hex + dump_c_float( profile.cell.Cmax );

    hex = hex + dump_c_float( profile.cell.Imid );
    hex = hex + dump_c_float( profile.cell.Cmid );

    hex = hex + dump_c_float( profile.cell.Vlow );
    hex = hex + dump_c_float( profile.cell.Clow );

    hex = hex + dump_c_float( profile.cell.Vmin );

    hex = hex + dump_c_float( profile.battery.Imax );
    hex = hex + dump_c_float( profile.battery.Pmax );

    hex = hex + dump_c_float( profile.battery.ESR );

    hex = hex + dump_c_uint8_t( profile.battery.parallel );
    hex = hex + dump_c_uint8_t( profile.battery.series   );

    hex = hex + dump_c_int( profile.display );
    hex = hex + dump_c_uint8_t( profile.allow_regen );

    return hex;
}
