
#include "MESCbat.h"

static BATProfile const * bat_profile;

static float grad_upper;    // Amp Hour per Volt
static float grad_lower;    // Amp Hour per Volt
static float Cscale;        // Charge scale factor (Cmax..Clow)

void bat_init( BATProfile const * profile )
{
    bat_profile = profile;

    float const u_dV = profile->Vmax - profile->Vmid;
    float const u_dC = profile->Cmax - profile->Cmid;

    grad_upper = u_dC / u_dV;

    float const l_dV = profile->Vmid - profile->Vmin;
    float const l_dC = profile->Cmid;

    grad_lower = l_dC / l_dV;

    Cscale = 100.0f / (profile->Cmax - profile->Clow); // Percent
    //Cscale = profile->Cmax / (profile->Cmax - profile->Clow); // Amp Hour
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

float bat_get_charge_level( float const V, float const I, float const ESR )
{
    float const Vadj = (V + (I * ESR));
    float dV;
    float  C;

    if (Vadj <= bat_profile->Vlow)
    {
        C = 0;
    }
    else if (Vadj >= bat_profile->Vmax)
    {
        C = bat_profile->Cmax;
    }
    else
    {
        if (Vadj > bat_profile->Vmid)
        {
            dV = (Vadj - bat_profile->Vmid);
            C  = bat_profile->Cmid + (grad_upper * dV);
        }
        else
        {
            dV = (Vadj - bat_profile->Vmin);
            C  = (grad_lower * dV);
        }

        if (bat_profile->Clow >= C)
        {
            C = 0;
        }
        else
        {
            C = (C - bat_profile->Clow);
        }
    }

    C = (C * Cscale);

    return C;
}

float bat_get_level_voltage( float const L )
{
    float Lrem = L;
    float Lmid = (bat_profile->Cmid - bat_profile->Clow) * Cscale;

    if (Lrem <= Lmid)
    {
        return (((Lrem * (bat_profile->Vmid - bat_profile->Vlow)) / Lmid) + bat_profile->Vlow);
    }

    Lrem = Lrem - Lmid;

    float Ltop = (bat_profile->Cmax - bat_profile->Cmid) * Cscale;

    return (((Lrem * (bat_profile->Vmax - bat_profile->Vmid)) / Ltop) + bat_profile->Vmid);
}
