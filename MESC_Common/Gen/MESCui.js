/*
* Copyright 2022 cod3b453
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

const UI_PROFILE_SIGNATURE = 'MUPE';

//UIProfileType
const UI_PROFILE_THROTTLE  = 0;
const UI_PROFILE_BRAKE     = 1;
const UI_PROFILE_BUTTON    = 2;

const UI_PROFILE_INDICATOR = 3;
const UI_PROFILE_SCREEN    = 4;
//end

const UI_PROFILE_SIZE = 32;

function dump_UIProfile_THROTTLE( profile ) {
    console.log( "dump_UIProfile_THROTTLE" );
    var hex = '';
    hex = hex + dump_c_uint32_t( profile.throttle_adc_min     );
    hex = hex + dump_c_uint32_t( profile.throttle_adc_max     );
    hex = hex + dump_c_int(      profile.throttle_response    );
    hex = hex + dump_c_uint32_t( profile.throttle_adc_trig    );
    hex = hex + dump_c_float(    profile.throttle_imax        );
    hex = hex + dump_c_uint32_t( profile.throttle_rcpwm_t_min );
    hex = hex + dump_c_uint32_t( profile.throttle_rcpwm_t_max );
    return hex;
}

function dump_UIProfile_BRAKE( profile ) {
    console.log( "dump_UIProfile_BRAKE" );
    var hex = '';
    hex = hex + dump_c_uint32_t( profile.brake_adc_min  );
    hex = hex + dump_c_uint32_t( profile.brake_adc_max  );
    hex = hex + dump_c_int(      profile.brake_response );
    hex = hex + dump_c_uint32_t( profile.brake_adc_trig );
    hex = hex + dump_c_float(    profile.brake_imax     );
    hex = hex + dump_c_char( 0, 4 );
    hex = hex + dump_c_char( 0, 4 );
    return hex;
}

function dump_UIProfile_BUTTON( profile ) {
    console.log( "dump_UIProfile_BUTTON" );
    var hex = '';
    hex = hex + dump_c_uint32_t( profile.button_interface  );
    hex = hex + dump_c_uint32_t( profile.button_address    );
    hex = hex + dump_c_uint32_t( profile.button_identifier );
    hex = hex + dump_c_char( 0, 4 );
    hex = hex + dump_c_char( 0, 4 );
    hex = hex + dump_c_char( 0, 4 );
    hex = hex + dump_c_char( 0, 4 );
    return hex;
}

function dump_UIProfile_INDICATOR( profile ) {
    console.log( "dump_UIProfile_INDICATOR" );
    var hex = '';
    hex = hex + dump_c_uint32_t( profile.indicator_interface  );
    hex = hex + dump_c_uint32_t( profile.indicator_address    );
    hex = hex + dump_c_uint32_t( profile.indicator_identifier );
    hex = hex + dump_c_int(      profile.indicator_activation );
    hex = hex + dump_c_char( 0, 4 );
    hex = hex + dump_c_char( 0, 4 );
    hex = hex + dump_c_char( 0, 4 );
    return hex;
}

function dump_UIProfile_SCREEN( profile ) {
    console.log( "dump_UIProfile_SCREEN" );
    var hex = '';
    hex = hex + dump_c_uint32_t( profile.screen_interface );
    hex = hex + dump_c_uint32_t( profile.screen_address   );
    hex = hex + dump_c_uint32_t( profile.screen_width     );
    hex = hex + dump_c_uint32_t( profile.screen_height    );
    hex = hex + dump_c_char( 0, 4 );
    hex = hex + dump_c_char( 0, 4 );
    hex = hex + dump_c_char( 0, 4 );
    return hex;
}

function dump_UIProfile( profile ) {
    console.log( "dump_UIProfile" );
    var hex = '';

    hex = hex + dump_c_int( profile.type );

    switch (profile.type) {
        case UI_PROFILE_THROTTLE : hex = hex + dump_UIProfile_THROTTLE(  profile ); break;
        case UI_PROFILE_BRAKE    : hex = hex + dump_UIProfile_BRAKE(     profile ); break;
        case UI_PROFILE_BUTTON   : hex = hex + dump_UIProfile_BUTTON(    profile ); break;
        case UI_PROFILE_INDICATOR: hex = hex + dump_UIProfile_INDICATOR( profile ); break;
        case UI_PROFILE_SCREEN   : hex = hex + dump_UIProfile_SCREEN(    profile ); break;
    }

    console.assert( hex.length == (NYBBLES_PER_BYTE * UI_PROFILE_SIZE) );

    return hex;
}

function UIProfile( type )
{
    this.type = type;

    this.throttle_adc_min = undefined;
    this.throttle_adc_max = undefined;
    this.throttle_response = undefined;
    this.throttle_adc_trig = undefined;
    this.throttle_adc_imax = undefined;
    this.throttle_rcpwm_t_min = undefined;
    this.throttle_rcpwm_t_max = undefined;

    this.brake_adc_min = undefined;
    this.brake_adc_max = undefined;
    this.brake_response = undefined;
    this.brake_adc_trig = undefined;
    this.brake_imax = undefined;

    this.button_interface = undefined;
    this.button_address = undefined;
    this.button_identifier = undefined;

    this.indicator_interface = undefined;
    this.indicator_address = undefined;
    this.indicator_identifier = undefined;
    this.indicator_activation = undefined;

    this.screen_interface = undefined;
    this.screen_address = undefined;
    this.screen_width = undefined;
    this.screen_height = undefined;
}

UIProfile.prototype.size = function()
{
    return UI_PROFILE_SIZE;
}

UIProfile.prototype.dump = function()
{
    return dump_UIProfile( this );
}
