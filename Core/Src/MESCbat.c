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

#include "MESCbat.h"

#include <stddef.h>

static BATProfile const * bat_profile = NULL;

static float grad_upper;    // Amp Hour per Volt
static float grad_lower;    // Amp Hour per Volt
static float Cscale;        // Charge scale factor (Cmax..Clow)

void bat_init( BATProfile const * const profile )
{
    bat_profile = profile;
    bat_notify_profile_update();

    float const u_dV = profile->cell.Vmax - profile->cell.Vmid;
    float const u_dC = profile->cell.Cmax - profile->cell.Cmid;

    grad_upper = u_dC / u_dV;

    float const l_dV = profile->cell.Vmid - profile->cell.Vmin;
    float const l_dC = profile->cell.Cmid;

    grad_lower = l_dC / l_dV;
}

void bat_notify_profile_update( void )
{
    switch (bat_profile->display)
    {
        case BAT_DISPLAY_PERCENT:
            Cscale = 100.0f / (bat_profile->cell.Cmax - bat_profile->cell.Clow);
            break;
        case BAT_DISPLAY_AMPHOUR:
            Cscale = bat_profile->cell.Cmax / (bat_profile->cell.Cmax - bat_profile->cell.Clow);
            break;
    }
}

float battery_get_power(
    float const Iq, float const Vq,
    float const Id, float const Vd )
{
    return ((Iq * Vq) + (Id * Vd));
}

float battery_get_current(
    float const Iq, float const Vq,
    float const Id, float const Vd,
    float const Vbat )
{
    float const power = battery_get_power( Iq, Vq, Id, Vd );
    return (power / Vbat);
}

float bat_get_charge_level( float const V, float const I)
{
    float const Vbat = (V + (I * bat_profile->battery.ESR));
    float const Vcell = (Vbat / bat_profile->battery.series);
    float dV;
    float  C;

    if (Vcell <= bat_profile->cell.Vlow)
    {
        C = 0;
    }
    else if (Vcell >= bat_profile->cell.Vmax)
    {
        C = bat_profile->cell.Cmax;
    }
    else
    {
        if (Vcell > bat_profile->cell.Vmid)
        {
            dV = (Vcell - bat_profile->cell.Vmid);
            C  = bat_profile->cell.Cmid + (grad_upper * dV);
        }
        else
        {
            dV = (Vcell - bat_profile->cell.Vmin);
            C  = (grad_lower * dV);
        }

        if (bat_profile->cell.Clow >= C)
        {
            C = 0;
        }
        else
        {
            C = (C - bat_profile->cell.Clow);
        }
    }

    C = (C * Cscale);

    return C;
}

float bat_get_level_voltage( float const L_C )
{
    float Lrem = L_C;
    float Lmid = (bat_profile->cell.Cmid - bat_profile->cell.Clow) * Cscale;

    if (Lrem <= Lmid)
    {
        return (((Lrem * (bat_profile->cell.Vmid - bat_profile->cell.Vlow)) / Lmid) + bat_profile->cell.Vlow);
    }

    Lrem = Lrem - Lmid;

    float Ltop = (bat_profile->cell.Cmax - bat_profile->cell.Cmid) * Cscale;

    return (((Lrem * (bat_profile->cell.Vmax - bat_profile->cell.Vmid)) / Ltop) + bat_profile->cell.Vmid);
}
