
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

float bat_get_voltage( void )
{
    return 0.0f;
}

float bat_get_charge_level( float const V )
{
    float dV;
    float  C;

    if (V <= bat_profile->Vlow)
    {
        C = 0;
    }
    else if (V >= bat_profile->Vmax)
    {
        C = bat_profile->Cmax;
    }
    else
    {
        if (V > bat_profile->Vmid)
        {
            dV = (V - bat_profile->Vmid);
            C  = bat_profile->Cmid + (grad_upper * dV);
        }
        else
        {
            dV = (V - bat_profile->Vmin);
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
