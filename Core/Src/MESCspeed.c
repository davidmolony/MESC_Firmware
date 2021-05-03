
#include "MESCspeed.h"

static float const Pi               = 3.14159265358979324f;
static float const seconds_per_hour = 3600.0f;

static float wheel_circumference; // (wheel units)
static float wheel_distance; // (wheel units) per (distance unit)
static float pole_pairs;
//static float factor = 6.0f;
static float speed; // (distance unit) per hour

static float gear_ratio; // gear_motor / gear_wheel

void speed_set_motor( uint32_t const pole_pairs_ )
{
    pole_pairs = (float)pole_pairs_;
}

void speed_set_wheel( float const diameter )
{
    wheel_circumference = Pi * diameter;
}

void speed_set_units( float const wheel_distance_ )
{
    wheel_distance = wheel_distance_;
}

void speed_set_gear_ratio(
    uint32_t const gear_motor,
    uint32_t const gear_wheel )
{
    gear_ratio = ((float)gear_motor) / ((float)gear_wheel);
}

float speed_get( void )
{
    float const dtheta = 0.0f; // revolutions // TODO
    float const dS = (dtheta * gear_ratio * wheel_circumference) / wheel_distance; // distamce
    float const dt = 0.0f; // time (seconds) // TODO

    float const v = ((seconds_per_hour * dS) / dt);

    speed = ((speed * 255.0f) + v) / 256.0f; // TODO

    return speed;
}
