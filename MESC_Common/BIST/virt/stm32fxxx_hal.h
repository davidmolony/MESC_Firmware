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

/*
Profile defaults
*/

/* Motor parameters */
#define DEFAULT_MOTOR_Ld          0.0f
#define DEFAULT_MOTOR_Lq          0.0f
#define DEFAULT_MOTOR_R           0.0f
#define DEFAULT_FLUX_LINKAGE      0.0f
#define MIN_FLUX_LINKAGE          0.0f
#define MAX_FLUX_LINKAGE          0.0f
#define FLUX_LINKAGE_GAIN         0.0f
#define NON_LINEAR_CENTERING_GAIN 0.0f

/* Temperature parameters */
#define MESC_PROFILE_TEMP_R_F     4700.0f                   // R_F 4k7
#define MESC_PROFILE_TEMP_SCHEMA  TEMP_SCHEMA_R_F_ON_R_T
#define MESC_PROFILE_TEMP_SH_BETA 3437.864258f
#define MESC_PROFILE_TEMP_SH_R    0.098243f
#define MESC_PROFILE_TEMP_SH_R0   10000.0f                  // R_T 10k [@ 25'C]
