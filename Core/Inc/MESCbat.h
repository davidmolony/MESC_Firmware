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
#if BAT_VTOP
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

float bat_get_level_voltage( float const L );

#endif
