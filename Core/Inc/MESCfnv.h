
#ifndef MESC_FNV1A_H
#define MESC_FNV1A_H

#include <stdint.h>

uint32_t fnv1a_data( void const * ptr, uint32_t const len );

uint32_t fnv1a_str( char const * ptr );

uint32_t fnv1a_init( void );

uint32_t fnv1a_process( uint32_t const fnv, uint8_t const byte );

uint32_t fnv1a_process_data( uint32_t const fnv, void const * ptr, uint32_t const len );

#endif
