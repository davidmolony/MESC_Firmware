
#ifndef MESC_SPEED_H
#define MESC_SPEED_H

#include <inttypes.h>

void speed_set_motor( uint32_t const pole_pairs );

void speed_set_wheel( float const diameter );

void speed_set_units( float const wheel_distance );

void speed_set_gear_ratio(
    uint32_t const gear_motor,
    uint32_t const gear_wheel );

float speed_get( void );

#endif
