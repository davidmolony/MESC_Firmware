
#include "MESCbat.h"

#include <stdio.h>

void bist_bat( void )
{
    fprintf( stdout, "Starting Battery BIST\n" );

    BATProfile bp;

    bp.Vmax = 4.2f; // V
    bp.Cmax = 4.2f; // Ah
#if 0
    bp.Vtop = 4.05f; // V
    bp.Ctop[0] = 4.0f; // Ah
    bp.Ctop[1] = 3.5f; // Ah
#endif
    bp.Vmid = 3.4f; // V
    bp.Cmid = 0.7f; // Ah

    bp.Vlow = 3.2f; // V
    bp.Clow = 0.5f; // Ah

    bp.Vmin = 2.8f; // V

    bat_init( &bp );

    for ( float V = bp.Vmin; (V < bp.Vmax); V = V + 0.05f )
    {
        float const C = bat_get_charge_level( V, 0.0f, 0.0f );

        fprintf( stdout, "%3.2f V => %3.0f %%\n", V, C ); // Percent
        //fprintf( stdout, "%3.2f V => %1.1f Ah\n", V, C ); // Amp Hour
    }

    fprintf( stdout, "Finished Battery BIST\n" );
}
