/*
* Copyright 2021-2023 cod3b453
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

#include "MESC_MOTOR_DEFAULTS.h"
// DANGER - Apply defaults before overriding 
#include "stm32fxxx_hal.h"
#include "math.h"
#include <stdint.h>

MOTORProfile * motor_profile = NULL;

void motor_init( MOTORProfile * const profile )
{
    if (profile == PROFILE_DEFAULT)
    {
        static MOTORProfile motor_profile_default;
		motor_profile_default.Imax = MAX_MOTOR_PHASE_CURRENT;
        motor_profile_default.Pmax = DEFAULT_MOTOR_POWER;
		motor_profile_default.pole_pairs =  DEFAULT_MOTOR_PP;
        motor_profile_default.L_D = DEFAULT_MOTOR_Ld;
        motor_profile_default.L_Q = DEFAULT_MOTOR_Lq;
        motor_profile_default.L_QD = DEFAULT_MOTOR_Lq-DEFAULT_MOTOR_Ld;
        motor_profile_default.R = DEFAULT_MOTOR_R;
		motor_profile_default.flux_linkage = DEFAULT_FLUX_LINKAGE;
		motor_profile_default.flux_linkage_min = MIN_FLUX_LINKAGE;
		motor_profile_default.flux_linkage_max = MAX_FLUX_LINKAGE;
		motor_profile_default.flux_linkage_gain = FLUX_LINKAGE_GAIN;
		motor_profile_default.non_linear_centering_gain = NON_LINEAR_CENTERING_GAIN;

        uint32_t            motor_length = sizeof(motor_profile_default);

        ProfileStatus const ret = profile_get_entry(
            "MTR", MOTOR_PROFILE_SIGNATURE,
            &motor_profile_default, &motor_length );

        motor_profile = &motor_profile_default;

        if (ret != PROFILE_STATUS_SUCCESS)
        {
            cli_reply( "MTR FAILED" "\r" "\n" );
        }
    }
    else
    {
    	motor_profile = profile;
    }
}

void speed_motor_limiter( void )
{
	motor_profile->Pmax = 50.0f; // Watts
}
