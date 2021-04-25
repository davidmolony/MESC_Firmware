
#ifndef MESC_TEMP_H
#define MESC_TEMP_H

#include <stdint.h>

float temp_read( uint32_t const adc_raw );

uint32_t temp_get_adc( float const T );

#endif
