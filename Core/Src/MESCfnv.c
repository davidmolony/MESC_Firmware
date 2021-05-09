
#include "MESCfnv.h"

//#include <inttypes.h>//debug
//#include <stdio.h>//debug

#define FNV1A_PRIME_32  UINT32_C(0x01000193)
#define FNV1A_OFFSET_32 UINT32_C(0x811C9DC5)

uint32_t fnv1a_process_data( uint32_t const fnv, void const * ptr, uint32_t const len )
{
    uint32_t fnv_ = fnv;

    for ( uint32_t i = 0; i < len; ++i )
    {
        fnv_ = fnv1a_process( fnv_, ((uint8_t const *)ptr)[i] );
    }

    return fnv_;
}

uint32_t fnv1a_data( void const * ptr, uint32_t const len )
{
    return fnv1a_process_data( FNV1A_OFFSET_32, ptr, len );
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
