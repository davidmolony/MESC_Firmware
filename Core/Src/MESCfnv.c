
#include "MESCfnv.h"

//#include <inttypes.h>//debug
//#include <stdio.h>//debug

#define FNV1A_PRIME_32  UINT32_C(0x01000193)
#define FNV1A_OFFSET_32 UINT32_C(0x811C9DC5)

uint32_t fnv1a_data( void const * ptr, uint32_t const len )
{
    uint32_t fnv = FNV1A_OFFSET_32;

    for ( uint32_t i = 0; i < len; ++i )
    {
        fnv ^= ((uint8_t const *)ptr)[i];
        fnv *= FNV1A_PRIME_32;
    }

    return fnv;
}

uint32_t fnv1a_str( char const * ptr )
{
    uint32_t hash = fnv1a_init();

    for ( char const * p = ptr; (*p != '\0'); ++p )
    {
        hash = fnv1a_process( hash, *p );
//fprintf( stderr, "HASH %08" PRIX32 "\n", hash );//debug
    }

    return hash;
}

uint32_t fnv1a_init( void )
{
    return FNV1A_OFFSET_32;
}

uint32_t fnv1a_process( uint32_t const fnv, uint8_t const byte )
{
    return ((fnv ^ byte) * FNV1A_PRIME_32);
}
