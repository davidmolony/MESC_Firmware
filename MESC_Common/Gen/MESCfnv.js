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

/*
REFERENCE

FNV-1A (Fowler/Noll/Vo) hash
http://www.isthe.com/chongo/tech/comp/fnv/index.html
Accessed 2021-04-17
*/

const FNV1A_PRIME_32  = 0x01000193;
const FNV1A_OFFSET_32 = 0x811C9DC5;

function fnv1a_init()
{
    return FNV1A_OFFSET_32;
}

function fnv1a_process( fnv, byte )
{
    var fnv_ = fnv ^ (byte & 0xFF);                 // FNV public domain
/*
DANGER

Must use >>> 0 to force unsigned
*/
    fnv_ = Math.imul( fnv_, FNV1A_PRIME_32 ) >>> 0; // FNV public domain
    fnV_ = (fnv_ & 0xFFFFFFFF);                     // FNV public domain

    return fnv_;
}

function fnv1a_process_hex( hex )
{
    var fnv = fnv1a_init();
    console.log( 'fnv1a_process_hex ' + fnv.toString(16) );
    for ( var i = 0; i < hex.length; i = i + 2 ) {
        var byte = parseInt( hex.substring(i,i+2), 16 );
        fnv = fnv1a_process( fnv, byte );
        console.log( 'fnv1a_process_hex[' + (i / 2).toString() + '] ' + byte.toString(16).toUpperCase() + ' ' + fnv.toString(16).toUpperCase() );
    }
    return fnv;
}
