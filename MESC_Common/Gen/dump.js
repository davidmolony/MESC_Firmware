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

var dump_log = 0;

function isString(o) {
    return typeof o == "string" || (typeof o == "object" && o.constructor === String);
}

function int_from_str(value,bytes) {
    var val = 0;
    for ( var b = bytes; b > 0; ) {
        b--;
        val = (val << BITS_PER_BYTE) | value.charCodeAt(b);
    }
    return val;
}

function toHex( value, bytes )
{
/*
DANGER

Must use >>> 0 to force unsigned
*/
    var hex = (value >>> 0).toString(16).toUpperCase().replace('-','');

    while (hex.length < (NYBBLES_PER_BYTE * bytes))
    {
        hex = "0" + hex;
    }

    return hex;
}

function dump_hex( value, bytes )
{
    dump_log++;

    var hex = toHex( value, bytes );
    var tmp = '';

    for ( var i = hex.length; i > 0; )
    {
        i = i - 2;
        tmp = tmp + hex.substring( i, i+2 );
    }
    if (dump_log == 1) {
        console.log( "dump_hex(value=" + value.toString() + "[" + value.toString(16).toUpperCase() + "],bytes=" + bytes.toString() + ") -> " + tmp );
    }
    dump_log--;
    return tmp;
}

function dump_base64( value, bytes )
{
    const BASE64_ENC = 'ABCDEFGHIJLKMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';
    const BITS_PER_SYMB = 6;
    var val = value;
    var hex = '';

    while (hex.length < (((BITS_PER_BYTE * bytes) + (BITS_PER_SYMB - 1)) / BITS_PER_SYMB))
    {
        hex = hex + BASE64_ENC.charAt( val & 0x3F );
        val = val >> 6;
    }
}

function dump_c_char( value, array_size = 1, pad_null = true )
{
    dump_log++;
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
    if (dump_log == 1) {
        console.log( "dump_c_char(value='" + value + "',array_size=" + array_size.toString() + ",pad_null=" + (pad_null ? "true" : "false" ) + ")" );
    }
    dump_log--;
    return hex;
}

function dump_c_int( value )
{
    dump_log++;
    var tmp = value;
    if (isString(value)) {
        tmp = int_from_str(value,4);
    }
    if (dump_log == 1) {
        console.log( "dump_c_int(value=" + value.toString() + "[" + tmp.toString(16).toUpperCase() + "])" );
    }
    var hex = dump_hex( (tmp & 0xFFFFFFFF), 4 );
    dump_log--;
    return hex;
}

function dump_c_float( value )
{
    dump_log++;
/*
REFERENCE

Convert float to 32bit hex string in JavaScript
Nina Scholz 2017-11-08
https://stackoverflow.com/a/47187116
Accessed 2021-12-12
*/
    const getHex = i => ('00' + i.toString(16).toUpperCase()).slice(-2);

    var view = new DataView(new ArrayBuffer(4)), result;

    view.setFloat32(0, value, true/*little-endian*/);

    result = Array
        .apply(null, { length: 4 })
        .map((_, i) => getHex(view.getUint8(i)))
        .join('');
    if (dump_log == 1) {
        console.log( "dump_c_float(value='" + value.toString() + "[" + result + "])" );
    }
    dump_log--;
    return result;
}

function dump_c_uint8_t( value )
{
    dump_log++;
    var tmp = value;
    if (isString(value)) {
        tmp = int_from_str(value,1);
    }
    var hex = dump_hex( (tmp & 0xFF), 1);
    if (dump_log == 1) {
        console.log( "dump_c_uint8_t(value=" + value.toString() + "[" + hex + "])" );
    }
    dump_log--;
    return hex;
}

function dump_c_uint16_t( value )
{
    dump_log++;
    var tmp = value;
    if (isString(value)) {
        tmp = int_from_str(value,2);
    }
    var hex = dump_hex( (tmp & 0xFFFF), 2);
    if (dump_log == 1) {
        console.log( "dump_c_uint16_t(value=" + value.toString() + "[" + hex + "])" );
    }
    dump_log--;
    return hex
}

function dump_c_uint32_t( value )
{
    dump_log++;
    var tmp = value;
    if (isString(value)) {
        tmp = int_from_str(value,4);
    }
    var hex = dump_hex( (tmp & 0xFFFFFFFF), 4 );
    if (dump_log == 1) {
        console.log( "dump_c_uint32_t(value=" + value.toString() + "[" + hex + "])" );
    }
    dump_log--;
    return hex;
}

function dump_MESCFingerprint()
{
    console.log( "dump_MESCFingerprint" );

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
    hex = hex + dump_c_char( '', 1 ); // NUL
    hex = hex + dump_c_char( '', 3 );
    hex = hex + GITHASH.toUpperCase();

    return hex;
}
