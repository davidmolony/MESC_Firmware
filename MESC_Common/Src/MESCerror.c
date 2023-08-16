
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

#include "MESCerror.h"
#include "MESCfoc.h"
#include "MESCmotor_state.h"
#include "MESChw_setup.h"


//externs
extern TIM_HandleTypeDef htim1;

//Variables
 struct MESC_log_vars error_log;
 uint32_t MESC_errors; //This is a bitwise uint32_t representation of the errors that have occurred.
 uint32_t MESC_all_errors; //All the errors since startup

void handleError(MESC_motor_typedef *_motor, uint32_t error_code){
	generateBreak(_motor); //Always generate a break when something bad happens
	if(_motor->MotorState == MOTOR_STATE_INITIALISING){
		MESC_errors|= (0b01<<(ERROR_STARTUP-1));
	}
	_motor->MotorState = MOTOR_STATE_ERROR;
	//Log the nature of the fault
	MESC_errors|= (0b01<<(error_code-1));
	if(error_log.count<1){ //only log the first error
	error_log.current_A = _motor->Conv.Iu;
	error_log.current_B = _motor->Conv.Iv;
	error_log.current_C = _motor->Conv.Iw;
	error_log.voltage = _motor->Conv.Vbus;
	error_log.motor_flux = _motor->m.flux_linkage;
	error_log.flux_a = _motor->FOC.flux_a;
	error_log.flux_b = _motor->FOC.flux_b;
	}
	error_log.count += 1;
	MESC_all_errors |= MESC_errors;
}

void clearErrors(){
	MESC_errors = 0;
	error_log.count = 0;
}

//Observe caution when using this function, BRK hypothetically occurs after a disastrous error.
void clearBRK(MESC_motor_typedef *_motor){
	//If the requested current is zero then sensible to proceed
	if((foc_vars.Idq_req.q+foc_vars.Idq_req.d)==0.0f){
	//Generate a break, and set the mode to tracking to enable a chance of safe restart and recovery
		generateBreak(_motor);
		//Need to set the MOE bit high to re-enable the timer
		htim1.Instance->BDTR |= (0b01);
		_motor->MotorState = MOTOR_STATE_TRACKING;
	}

}
