/*
 **
 ******************************************************************************
 * @file           : MESCsin_lut.h
 * @brief          : BLDC running code
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

 * MESCsin_lut.h
 *
 *  Created on: 25 Jul 2020
 *      Author: David Molony
 */
#include <stdint.h>

// Vector 256 long of sin wave, stretched by 65 to allow computation of cosine as sin(angle+64) without needing wrapping
extern float sinwave[321];
