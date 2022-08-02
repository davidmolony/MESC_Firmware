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

#include "MESCmotor.h"

#include "MESCcli.h"
#include "MESCprofile.h"

#include "stm32fxxx_hal.h"

#include <stdint.h>

MOTORProfile * motor_profile = NULL;

void motor_init( MOTORProfile const * const profile )
{
    if (profile == PROFILE_DEFAULT)
    {
        static MOTORProfile motor_profile_default =
        {
        	.Imax = MAX_IQ_REQUEST, // A
        	.Vmax =  88.0f, // V
			.Pmax = DEFAULT_MOTOR_POWER, // W
			.RPMmax = 100000, // eRPM
			.pole_pairs = POLE_PAIRS,//7
			.direction = 0,
			.L_D = DEFAULT_FLUX_LINKAGE,
			.L_Q = DEFAULT_MOTOR_Lq,
			.R   = DEFAULT_MOTOR_R,
			.flux_linkage = DEFAULT_FLUX_LINKAGE,
			.flux_linkage_min = MAX_FLUX_LINKAGE,
			.flux_linkage_max = MIN_FLUX_LINKAGE,
			.flux_linkage_gain = FLUX_LINKAGE_GAIN,
			.non_linear_centering_gain = NON_LINEAR_CENTERING_GAIN,

			.sensor =
			{
				.encoder_offset = ENCODER_E_OFFSET,
				.hall_states =
				{
					{
						.min   = 10922,
						.max   = 21845,
						.mid   = 16384,
						.width = 10922,
					},
					{
						.min   = 32768,
						.max   = 43690,
						.mid   = 38229,
						.width = 10922,
					},
					{
						.min   = 21845,
						.max   = 32768,
						.mid   = 27307,
						.width = 10922,
					},
					{
						.min   = 54613,
						.max   = 65535,
						.mid   = 60075,
						.width = 10922,
					},
					{
						.min   =     0,
						.max   = 10922,
						.mid   =  5461,
						.width = 10922,
					},
					{
						.min   = 43691,
						.max   = 54613,
						.mid   = 49152,
						.width = 10922,
					},
				},
			},
        };
        static uint32_t motor_length = sizeof(motor_profile_default);

        ProfileStatus const ret = profile_get_entry(
            "MTR", MOTOR_PROFILE_SIGNATURE,
            &motor_profile_default, &motor_length );

        motor_profile = &motor_profile_default;

        if (ret != PROFILE_STATUS_SUCCESS)
        {
            cli_reply( "MTR FAILED" "\r" "\n" );

            profile_alloc_entry( "MTR", MOTOR_PROFILE_SIGNATURE, &motor_profile_default, &motor_length );
        }
    }
    else
    {
    	motor_profile = profile;
    }
}
