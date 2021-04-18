
#include "MESCfnv.h"

#define FNV1A_PRIME_32  UINT32_C(0x01000193)
#define FNV1A_OFFSET_32 UINT32_C(0x811C9DC5)

uint32_t fnv1a( void const * ptr, uint32_t const len )
{
    uint32_t fnv = FNV1A_OFFSET_32;

    for ( uint32_t i = 0; i < len; ++i )
    {
        fnv ^= ((uint8_t const *)ptr)[i];
        fnv *= FNV1A_PRIME_32;
    }

    return fnv;
}

uint32_t fnv1a_init( void )
{
    return FNV1A_OFFSET_32;
}

uint32_t fnv1a_process( uint32_t const fnv, uint8_t const byte )
{
    return ((fnv ^ byte) * FNV1A_PRIME_32);
}
