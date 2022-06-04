/*
* Copyright 2021-2022 cod3b453
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

    bp.cell.Imax = 30.0f; // A
    bp.cell.Vmax =  4.2f; // V
    bp.cell.Cmax =  4.2f; // Ah

    bp.cell.Vmid = 3.4f; // V
    bp.cell.Cmid = 0.7f; // Ah

    bp.cell.Vlow = 3.2f; // V
    bp.cell.Clow = 0.5f; // Ah

    bp.cell.Vmin = 2.8f; // V

    bp.battery.Imax =  50.0f; // A (fuse)
    bp.battery.Pmax = 250.0f; // W

    bp.battery.parallel =  2;
    bp.battery.series   = 20;

    bp.battery.ESR  = ((0.030f/*Cell ESR*/ * (float)bp.battery.series) / (float)bp.battery.parallel);

    float const Vmax = bp.cell.Vmax * (float)bp.battery.series;
    float const Vdel =        0.05f * (float)bp.battery.series;
    float const Vlow = bp.cell.Vlow * (float)bp.battery.series;

    float const Cmax = bp.cell.Cmax * (float)bp.battery.parallel;
    float const Cdel =         0.5f * (float)bp.battery.parallel;
    float const Clow = bp.cell.Clow * (float)bp.battery.parallel;

    for ( BATDisplay d = BAT_DISPLAY_PERCENT; (d <= BAT_DISPLAY_AMPHOUR); ++d )
    {
        fprintf( stdout, "Display %d\n", d );

        bp.display = d;

        bat_init( &bp ); // NOTE calls bat_notify_profile_update

        for ( float V = Vlow; (V <= Vmax); V = V + Vdel )
        {
            float const C = bat_get_charge_level( V, 0.0f );

            switch (bp.display)
            {
                case BAT_DISPLAY_PERCENT:
                    fprintf( stdout, "    %3.2f V => %3.0f %%\n", V, C );
                    break;
                case BAT_DISPLAY_AMPHOUR:
                    fprintf( stdout, "    %3.2f V => %5.1f Ah\n", V, C );
                    break;
            }
        }

        switch (bp.display)
        {
            case BAT_DISPLAY_PERCENT:
                for ( float L = 0.0f; (L <= 100.0f); L = L + 10.0f )
                {
                    float const V = bat_get_level_voltage( L );

                    fprintf( stdout, "    %3.0f %% => %5.2f V\n", L, V );
                }
                break;
            case BAT_DISPLAY_AMPHOUR:
                for ( float C = Clow; (C <= Cmax); C = C + Cdel )
                {
                    float const V = bat_get_level_voltage( C );

                    fprintf( stdout, "    %5.1f Ah => %5.2f V\n", C, V );
                }
                break;
        }
    }

    fprintf( stdout, "Finished Battery BIST\n" );
}
