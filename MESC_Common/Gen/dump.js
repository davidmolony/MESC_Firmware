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

function dump_hex( value, bytes )
{
    var hex = value.toString(16).toUpperCase().replace('-','');

    while (hex.length < (NYBBLES_PER_BYTE * bytes))
    {
        hex = "0" + hex;
    }

    return hex;
}

function dump_c_char( value, array_size = 1, pad_null = true )
{
    var hex = '';
    let i = 0;

    for ( ; ((i < value.length) && array_size); i++ )
    {
        hex = hex + dump_c_uint8_t( value.charCodeAt(i) );
    }

    if (pad_null)
    {
        for ( ; i < array_size; i++ )
        {
            hex = hex + dump_c_uint8_t( 0 );
        }
    }

    return hex;
}

function dump_c_int( value )
{
    return dump_hex( (value & 0xFFFFFFFF), 4 );
}

function dump_c_float( value )
{
/*
REFERENCE

Convert float to 32bit hex string in JavaScript
Nina Scholz 2017-11-08
https://stackoverflow.com/a/47187116
Accessed 2021-12-12
*/
    const getHex = i => ('00' + i.toString(16)).slice(-2);

    var view = new DataView(new ArrayBuffer(4)), result;

    view.setFloat32(0, value);

    result = Array
        .apply(null, { length: 4 })
        .map((_, i) => getHex(view.getUint8(i)))
        .join('');

    return result;
}

function dump_c_uint8_t( value )
{
    return dump_hex( (value & 0xFF), 1);
}

function dump_c_uint16_t( value )
{
    return dump_hex( (value & 0xFFFF), 2);
}

function dump_c_uint32_t( value )
{
    return dump_hex( (value & 0xFFFFFFFF), 4 );
}

function dump_MESCFingerprint()
{
    var hex = '';
/*
    // YYYYMMDDhhmmss
    var DATETIME = new Date().toISOString().slice(-24).replace(/\D/g,'').slice(0, 14);
*//*
    var DATE_Y = parseInt( DATETIME.substring( 0, 4) );
    var DATE_M = parseInt( DATETIME.substring( 4, 6) );
    var DATE_D = parseInt( DATETIME.substring( 6, 8) );
    var TIME_H = parseInt( DATETIME.substring( 8,10) );
    var TIME_M = parseInt( DATETIME.substring(10,12) );
    var TIME_S = parseInt( DATETIME.substring(12,14) );
*/
//  hex = hex + dump_c_char( DATETIME, (4/*YYYY*/+2/*MM*/+2/*DD*/)+(2/*hh*/+2/*mm*/)+1/*NUL*/ );

    hex = hex + dump_c_char( TIMESTAMP );
    hex = hex + dump_c_char( '', 1 );
    hex = hex + dump_c_char( '', 3 );
    hex = hex + GITHASH.toUpperCase();

    return hex;
}
