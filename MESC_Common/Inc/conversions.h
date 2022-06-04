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

#ifndef MESC_CONVERSIONS_H
#define MESC_CONVERSIONS_H

/*
CONSTANTS
*/

/*
Distance
*/

#define CONST_CENTIMETRES_PER_INCH_F                (2.54f)

#define CONST_INCHES_PER_MILE_U             UINT32_C(63360)
#define CONST_INCHES_PER_MILE_F                     (63360.0f)

#define CONST_INCHES_PER_KILOMETRE_U        UINT32_C(39370)
#define CONST_INCHES_PER_KILOMETRE_F                (39370.1f)

#define CONST_CENTIMETRES_PER_MILE_U        UINT32_C(160934)
#define CONST_CENTIMETRES_PER_MILE_F                (160934.0f)

#define CONST_CENTIMETRES_PER_KILOMETRE_U   UINT32_C(100000)
#define CONST_CENTIMETRES_PER_KILOMETRE_F           (100000.0f)

/*
Mathematical
*/

/*
SQRT(2)
*/
#define CONST_SQRT_2_F                      (1.41421356237f)

/*
   1
-------
SQRT(2)
*/
#define CONST_1_R_SQRT_2_F                  (0.70710678118f)

/*
    / 2 \
SQRT| - |
    \ 3 /
*/
#define CONST_SQRT_2_R_3_F                  (0.81649658092f)

/*
    / 3 \
SQRT| - |
    \ 2 /
*/
#define CONST_SQRT_3_R_2_F                  (1.22474487139f)

/*
   1
-------
SQRT(3)
*/
#define CONST_1_R_SQRT_3_F                  (0.57735026919f)

/*
   1
-------
SQRT(6)
*/
#define CONST_1_R_SQRT_6_F                  (0.40824829046f)

/*
1
-
3
*/
#define CONST_1_R_3_F                       (0.33333333333f)

/*
Temperature
*/

#define CONST_ZERO_CELSIUS_TO_ABSOLUTE_U    UINT32_C( 273)
#define CONST_ZERO_CELSIUS_TO_ABSOLUTE_F            ( 273.15f)

#define CONST_ABSOLUTE_ZERO_TO_CELSIUS_I     INT32_C(-273)
#define CONST_ABSOLUTE_ZERO_TO_CELSIUS_F            (-273.15f)

/*
Time
*/

#define CONST_SECONDS_PER_MINUTE_U          UINT32_C(60)
#define CONST_SECONDS_PER_MINUTE_F                  (60.0f)

#define CONST_MINUTES_PER_HOUR_U            UINT32_C(60)
#define CONST_MINUTES_PER_HOUR_F                    (60.0f)

#define CONST_SECONDS_PER_HOUR_U            UINT32_C(3600)      // CONST_SECONDS_PER_MINUTE_U * CONST_MINUTES_PER_HOUR_U
#define CONST_SECONDS_PER_HOUR_F                    (3600.0f)   // CONST_SECONDS_PER_MINUTE_F * CONST_MINUTES_PER_HOUR_F

/*
Trigonometric
*/

#define CONST_PI_F   (3.14159274f)
#define CONST_2_PI_F (6.28318531f)

/*
CONVERSIONS
*/

/*
Temperature
*/

#define CVT_KELVIN_TO_CELSIUS_U(K)  (CONST_ABSOLUTE_ZERO_TO_CELSIUS_U + (K))
#define CVT_KELVIN_TO_CELSIUS_F(K)  (CONST_ABSOLUTE_ZERO_TO_CELSIUS_F + (K))

#define CVT_CELSIUS_TO_KELVIN_U(C)  (CONST_ZERO_CELSIUS_TO_ABSOLUTE_U + (C))
#define CVT_CELSIUS_TO_KELVIN_F(C)  (CONST_ZERO_CELSIUS_TO_ABSOLUTE_F + (C))

#endif
