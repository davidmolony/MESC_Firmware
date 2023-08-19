
/*
* Copyright 2021-2022 David Molony
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
#ifndef INC_MESCERROR_H_
#define INC_MESCERROR_H_

//Includes
#include "stm32fxxx_hal.h"
#include "MESCfoc.h"

//Variables
extern  uint32_t MESC_errors;
extern  uint32_t MESC_all_errors;

struct MESC_log_vars
{
	float current_A;
	float current_B;
	float current_C;
	float voltage;
	float motor_flux;
	float flux_a;
	float flux_b;
	int count;
};



//Define anticipated errors possible, not all will be implemented
#define ERROR_OVERCURRENT_PHA 1
#define ERROR_OVERCURRENT_PHB 2
#define ERROR_OVERCURRENT_PHC 3
#define ERROR_OVERVOLTAGE 4
#define ERROR_UNDERVOLTAGE 5
#define ERROR_BRK 6
#define ERROR_OVERTEMPU 7
#define ERROR_OVERTEMPV 8
#define ERROR_OVERTEMPW 9
#define ERROR_OVERTEMP_MOTOR 10
#define ERROR_HARDFAULT 11
#define ERROR_BUSFAULT 12
#define ERROR_NMI 13
#define ERROR_MEMFAULT 14
#define ERROR_USAGE 15
#define ERROR_ADC_OUT_OF_RANGE_IA 16
#define ERROR_ADC_OUT_OF_RANGE_IB 17
#define ERROR_ADC_OUT_OF_RANGE_IC 18
#define ERROR_ADC_OUT_OF_RANGE_VBUS 19
#define ERROR_WDG 20
#define ERROR_UNBALANCED_CURRENT 21
#define ERROR_MEASUREMENT_FAIL 22
#define ERROR_DETECTION_FAIL 23
#define ERROR_HALL0 24
#define ERROR_HALL7 25
#define ERROR_MATH 26
#define ERROR_INPUT_OOR 27
#define ERROR_STARTUP 28

void handleError(MESC_motor_typedef *_motor, uint32_t error_code);

#endif /* INC_MESCERROR_H_ */
