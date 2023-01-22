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

/*
REFERENCE

FNV-1A (Fowler/Noll/Vo) hash
http://www.isthe.com/chongo/tech/comp/fnv/index.html
Accessed 2021-04-17

In line with the license of this reference source, lines marked:

    // FNV public domain

remain in the public domain and are not bound by any other copyright or license
in this file or repository.
*/

#include "include/TTerm_fnv.h"

//#include <inttypes.h>//debug
//#include <stdio.h>//debug

#define FNV1A_PRIME_32  UINT32_C(0x01000193)    // FNV public domain
#define FNV1A_OFFSET_32 UINT32_C(0x811C9DC5)    // FNV public domain

uint32_t TTERM_fnv1a_process_data( uint32_t const fnv, void const * ptr, uint32_t const len )
{
    uint32_t fnv_ = fnv;

    for ( uint32_t i = 0; i < len; ++i )
    {
        fnv_ = TTERM_fnv1a_process( fnv_, ((uint8_t const *)ptr)[i] );
    }

    return fnv_;
}

uint32_t TTERM_fnv1a_data( void const * ptr, uint32_t const len )
{
    return TTERM_fnv1a_process_data( FNV1A_OFFSET_32, ptr, len );
}

uint32_t TTERM_fnv1a_str( char const * ptr )
{
    uint32_t hash = TTERM_fnv1a_init();

    for ( char const * p = ptr; (*p != '\0'); ++p )
    {
        hash = TTERM_fnv1a_process( hash, *p );
    }

    return hash;
}

uint32_t TTERM_fnv1a_init( void )
{
    return FNV1A_OFFSET_32;
}

uint32_t TTERM_fnv1a_process( uint32_t const fnv, uint8_t const byte )
{
    return ((fnv ^ byte) * FNV1A_PRIME_32);     // FNV public domain
}

uint32_t TTERM_fnv1a_process_zero( uint32_t const fnv, uint32_t const len )
{
    uint32_t fnv_ = fnv;

    for ( uint32_t i = 0; i < len; ++i )
    {
        fnv_ = TTERM_fnv1a_process( fnv_, 0 );
    }

    return fnv_;
}
