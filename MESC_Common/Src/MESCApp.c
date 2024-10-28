/*
 **
 ******************************************************************************
 * @file           : MESCApp.h
 * @brief          : Application specific code, such as for managing vehicle inputs for brake, stands...
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 David Molony.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 *In addition to the usual 3 BSD clauses, it is explicitly noted that you
 *do NOT have the right to take sections of this code for other projects
 *without attribution and credit to the source. Specifically, if you copy into
 *copyleft licenced code without attribution and retention of the permissive BSD
 *3 clause licence, you grant a perpetual licence to do the same regarding turning sections of your code
 *permissive, and lose any rights to use of this code previously granted or assumed.
 *
 *This code is intended to remain permissively licensed wherever it goes,
 *maintaining the freedom to distribute compiled binaries WITHOUT a requirement to supply source.
 *
 *This is to ensure this code can at any point be used commercially, on products that may require
 *such restriction to meet regulatory requirements, or to avoid damage to hardware, or to ensure
 *warranties can reasonably be honoured.
 ******************************************************************************
 * MESCfoc.h
 *
 *  Created on: Oct 2024
 *      Author: David Molony
 */

#include "MESCApp.h""
#include "MESCfoc.h"
#include "MESCmotor_state.h"
#include "MESCerror.h"
#include "stm32fxxx_hal.h"

typedef struct {
	uint32_t reverseState;
	uint32_t sidestandState;
	uint32_t brakeState;

}Vehicle_s;

Vehicle_s vehicle;
#define getReverseState(...) ((REVERSE_GPIO->IDR >> (REVERSE_IONO)) & 0x1)
#define getSidestandState(...) ((SIDESTAND_GPIO->IDR >> (SIDESTAND_IONO)) & 0x1)
#define getBrakeState(...) ((BRAKE_GPIO->IDR >> (BRAKE_IONO)) & 0x1)

uint32_t vehicle_state = VEHICLE_IDLE;
uint32_t vehicle_direction = VEHICLE_FORWARD;
uint32_t debounce(uint32_t var, uint32_t input){
	if((var>0) && (!input)){
		 var = var-1;
	}else if(var == 0){
		var = 100; //Reset it to far from the threshold
	}else if((var<100)){
		var = var+1;
	}
	return var;
}
void No_app(MESC_motor_typedef *_motor){
	//Sum the inputs
	  //We just scale and sum the input current requests
	  _motor->FOC.Idq_prereq.q = 	_motor->input_vars.UART_req +
	  	  	  	  	  	  	  	  	_motor->input_vars.max_request_Idq.q * (_motor->input_vars.ADC1_req + _motor->input_vars.ADC2_req +
	  	  	  	  	  	  	  	  	_motor->input_vars.RCPWM_req + _motor->input_vars.ADC12_diff_req +
									_motor->input_vars.remote_ADC1_req + _motor->input_vars.remote_ADC2_req );

	  //Clamp the Q component; d component is not directly requested
	  _motor->FOC.Idq_prereq.q = clamp(_motor->FOC.Idq_prereq.q, _motor->input_vars.min_request_Idq.q, _motor->input_vars.max_request_Idq.q);
	  vehicle.sidestandState = getSidestandState();
#ifdef MOMENTARY_REV
//	  vehicle.reverseState = getReverseState();
	  vehicle.reverseState = debounce(vehicle.reverseState, getReverseState());
	  if((!vehicle.reverseState)&&(fabsf(_motor->FOC.Idq_prereq.q)<5.0f)){
		  vehicle_direction = vehicle_direction+1;
		  vehicle_direction = vehicle_direction & 0x1;
	  }
	  if(vehicle_direction){
		  //Nothing
	  }else{
		  _motor->FOC.Idq_prereq.q *= -1.0f;
	  }
#else
	  vehicle.reverseState = getReverseState();
	  if(vehicle.reverseState == 0){
		  _motor->FOC.Idq_prereq.q *= -1.0f;
	  }
#endif
	  vehicle.brakeState = getBrakeState();

}

void Vehicle_app(MESC_motor_typedef *_motor){

	switch(vehicle_state){
	case VEHICLE_IDLE:
		_motor->key_bits |= APP_KEY; //Set the bit; lock the motor from current
//Add state machine logics
		break;
	case VEHICLE_PARKED:
		_motor->key_bits |= APP_KEY; //Set the bit; lock the motor from current
//Add state machine logics

		break;
	case VEHICLE_DRIVE:
		_motor->key_bits &= ~APP_KEY; //Clear the bit; allow motor current
//Add state machine logics

		break;
	case VEHICLE_x:
		break;
	case VEHICLE_y:
		break;
	default:
		handleError(_motor,ERROR_APP);
		_motor->key_bits |= APP_KEY; //Set the bit; lock the motor from current
	}

//	_motor->key_bits;

// 	_motor->key_bits |= APP_KEY; Set the bit; lock the motor from current
//	_motor->key_bits &= ~APP_KEY; Clear the bit - allow current


}
