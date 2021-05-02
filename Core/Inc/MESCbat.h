
#ifndef MESC_BAT_H
#define MESC_BAT_H

#include <stdint.h>

/*
Battery Profile

 V
4.2-+ - - - - - - - Vmax
4.0 | \__ _ _ _ _ _ Vtop
    |    \
    |     \
    |      \
3.4-+ - - - | - - - Vmid
3.2-+ - - - | - - - Vlow
2.8-+ - - - - - - - Vmin
    :       :
----+--------------
   Cmax    Cmin     Ah

Vmax Maximum charged voltage
Vtop Region where discharge voltage is flat
Vmid Point where linear discharge transfers to roll-off
Vlow Point of maximum safe discharge
Vmin Point of absolute discharge

Charge level derivation uses two stages of linear approximation from Vmax to
Vmid and Vmid to Vmin. (Vtop region is ignored) These values are rebased on Vlow
to reduce adverse impact on battery condition due to over discharge.
*/

struct BATProfile
{
    float   Vmax;   /* Volt     */
    float   Cmax;   /* Amp Hour */
#if 0
    float   Vtop;   /* Volt     */
    float   Ctop[2];/* Amp Hour */
#endif
    float   Vmid;   /* Volt     */
    float   Cmid;   /* Amp Hour */

    float   Vlow;   /* Volt     */
    float   Clow;   /* Amp Hour */

    float   Vmin;   /* Volt     */
};

typedef struct BATProfile BATProfile;

void bat_init( BATProfile const * profile );

float battery_get_power(
    float const Iq, float const Vq,
    float const Id, float const Vd );

float battery_get_current(
    float const Iq, float const Vq,
    float const Id, float const Vd,
    float const Vbat );

float bat_get_charge_level( float const V, float const I, float const ESR );

#endif
