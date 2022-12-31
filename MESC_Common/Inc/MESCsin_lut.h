/*
 **
 ******************************************************************************
 * @file           : MESCsin_lut.c
 * @brief          : Sine LUT
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 David Molony.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************

 * MESCsin_lut.c
 *
 *  Created on: 11 Apr 2022
 *      Author: David Molony
 *      		Jens Kerrinnes - Added higher res sin_lut
 *
 */

#include <stdint.h>

#define USE_HIGH_RES	1 //Seems to use an extra 30 clock cycles or so

void sin_cos_fast( uint16_t angle , float * sin, float * cos);

void getLabFast( uint16_t angle, float Ld, float Lq_Ld , float * La, float * Lb);
