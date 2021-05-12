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

#include <stdio.h>

void bist_bat( void )
{
    fprintf( stdout, "Starting Battery BIST\n" );

    BATProfile bp;

    bp.Vmax = 4.2f; // V
    bp.Cmax = 4.2f; // Ah
#if BAT_VTOP
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

    for ( float L = 0.0f; (L <= 100.0f); L = L + 10.0f )
    {
        float const V = bat_get_level_voltage( L );

        fprintf( stdout, "%3.0f %% => %3.2f V\n", L, V ); // Percent
    }

    fprintf( stdout, "Finished Battery BIST\n" );
}
