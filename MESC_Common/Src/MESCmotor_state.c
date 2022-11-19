/*
 **
 ******************************************************************************
 * @file           : MESCmotor_state.c
 * @brief          : Code for motor state machine
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

 * MESCmotor_state.c
 *
 *  Created on: 25 Jul 2020
 *      Author: David Molony
 */

/* Includes ------------------------------------------------------------------*/
#include "MESCmotor_state.h"

motor_state_e MotorState;
motor_sensor_mode_e MotorSensorMode;
motor_error_type_e MotorError;
motor_direction_e MotorDirection;
motor_control_type_e MotorControlType;
test_mode_e TestMode;


void MESCmotor_state_set(motor_state_e mState){

	MotorState = mState;

	  switch (MotorState) {

	  	case MOTOR_STATE_INITIALISING:
	  		break;

	    case MOTOR_STATE_RUN:
	    	generateEnable();
	      break;
	    case MOTOR_STATE_TRACKING:
	#ifdef HAS_PHASE_SENSORS
	    	generateBreak();
	#endif
	      break;

	    case MOTOR_STATE_OPEN_LOOP_STARTUP:
	      break;
	    case MOTOR_STATE_OPEN_LOOP_TRANSITION:
	      break;
	    case MOTOR_STATE_IDLE:
	        generateBreak();
	      // Do basically nothing
	      break;
	    case MOTOR_STATE_DETECTING:
	      break;
	    case MOTOR_STATE_MEASURING:
	      break;
	    case MOTOR_STATE_GET_KV:
	      break;
	    case MOTOR_STATE_ERROR:
	      generateBreak();  // Generate a break state (software disabling all PWM)
	                        // Now panic and freak out
	      break;
	    case MOTOR_STATE_ALIGN:
	      break;
	    case MOTOR_STATE_TEST:
	      break;
	    case MOTOR_STATE_RECOVERING:
	      break;
	    default:
	      MotorState = MOTOR_STATE_ERROR;
	      generateBreak();
	      break;
	  }

}


