/*
 **
 ******************************************************************************
 * @file           : MESCinput.c
 * @brief          : Collect inputs like throttle/brake
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
 */
#include <math.h>
#include "MESCinput.h"
#include "MESCerror.h"


void MESCinput_Init(MESC_motor_typedef *_motor){

	_motor->input_vars.max_request_Idq.d = 0.0f; //Not supporting d-axis input current for now
	_motor->input_vars.min_request_Idq.d = 0.0f;
	if(!_motor->input_vars.max_request_Idq.q){
		_motor->input_vars.max_request_Idq.q = MAX_IQ_REQUEST;
		_motor->input_vars.min_request_Idq.q = MIN_IQ_REQUEST; //ToDo, SETTING THESE ASSYMETRIC WILL CAUSE ISSUES WITH REVERSE..
	}

	_motor->input_vars.IC_pulse_MAX = IC_PULSE_MAX;
	_motor->input_vars.IC_pulse_MIN = IC_PULSE_MIN;
	_motor->input_vars.IC_pulse_MID = IC_PULSE_MID;
	_motor->input_vars.IC_pulse_DEADZONE = IC_PULSE_DEADZONE;
	_motor->input_vars.IC_duration_MAX = IC_DURATION_MAX;
	_motor->input_vars.IC_duration_MIN = IC_DURATION_MIN;

	if(!_motor->input_vars.adc1_MAX){
		_motor->input_vars.adc1_MAX = ADC1MAX;
		_motor->input_vars.adc1_MIN = ADC1MIN;
		_motor->input_vars.adc1_OOR = ADC1OOR;
		_motor->input_vars.ADC1_polarity = ADC1_POLARITY;
	}
	if(!_motor->input_vars.adc2_MAX){
		_motor->input_vars.adc2_MAX = ADC2MAX;
		_motor->input_vars.adc2_MIN = ADC2MIN;
		_motor->input_vars.adc2_OOR = ADC2OOR;
		_motor->input_vars.ADC2_polarity = ADC2_POLARITY;
	}

	_motor->input_vars.adc1_gain[0] = 1.0f/(_motor->input_vars.adc1_MAX-_motor->input_vars.adc1_MIN);
	_motor->input_vars.adc1_gain[1] = 1.0f/(_motor->input_vars.adc1_MAX-_motor->input_vars.adc1_MIN);

	_motor->input_vars.adc2_gain[0] = 1.0f/(_motor->input_vars.adc2_MAX-_motor->input_vars.adc2_MIN);
	_motor->input_vars.adc2_gain[1] = 1.0f/(_motor->input_vars.adc2_MAX-_motor->input_vars.adc2_MIN);

	//RCPWM forward gain//index [0][x] is used for Idq requests for now, might support asymmetric brake and throttle later
	_motor->input_vars.RCPWM_gain[0][0] = 1.0f/((float)_motor->input_vars.IC_pulse_MAX - (float)_motor->input_vars.IC_pulse_MID - (float)_motor->input_vars.IC_pulse_DEADZONE);
	_motor->input_vars.RCPWM_gain[0][1] = 1.0f/(((float)_motor->input_vars.IC_pulse_MID - (float)_motor->input_vars.IC_pulse_DEADZONE)-(float)_motor->input_vars.IC_pulse_MIN);

	if(!_motor->input_vars.input_options){
		_motor->input_vars.input_options = DEFAULT_INPUT;
	}

	_motor->input_vars.UART_req = 0.0f;
	_motor->input_vars.RCPWM_req = 0.0f;
	_motor->input_vars.ADC1_req = 0.0f;
	_motor->input_vars.ADC2_req = 0.0f;
}

void MESCinput_Collect(MESC_motor_typedef *_motor){

	  //Check if remote ADC timeouts
	  if(_motor->input_vars.remote_ADC_timeout > 0){
		  _motor->input_vars.remote_ADC_timeout--;
	  }else{
		  _motor->input_vars.remote_ADC1_req = 0.0f;
		  _motor->input_vars.remote_ADC2_req = 0.0f;
	  }

	  //Collect the requested throttle inputs

	  //Remote ADC1 input
	  if((_motor->input_vars.input_options & 0b100000)&&(_motor->input_vars.remote_ADC_can_id > 0)){
		  	  //Do nothing. Already set
	  }else{
		  _motor->input_vars.remote_ADC1_req = 0.0f;//Set the input variable to zero
	  }

	  //Remote ADC2 input
	  if((_motor->input_vars.input_options & 0b1000000)&&(_motor->input_vars.remote_ADC_can_id > 0)){
		  //Do nothing, already set
	  }else{
		  _motor->input_vars.remote_ADC2_req = 0.0f;//Set the input variable to zero
	  }

	  //Differential ADC12 input
	  if((_motor->input_vars.input_options & 0b10000000)){
		  //TBC, Math and logic required
		  //To be filled, as signal = ext1-ext2 with error check based on ext1+ext2
	  }else{
		  _motor->input_vars.ADC12_diff_req = 0.0f; //Set the input variable to zero
	  }

	  //UART input
	  if(0 == (_motor->input_vars.input_options & 0b1000)){
		  _motor->input_vars.UART_req = 0.0f;
	  }

	  //RCPWM input
	  if(_motor->input_vars.input_options & 0b0100){
		  if(_motor->input_vars.pulse_recieved){
			  if((_motor->input_vars.IC_duration > _motor->input_vars.IC_duration_MIN) && (_motor->input_vars.IC_duration < _motor->input_vars.IC_duration_MAX)){
				  if(_motor->input_vars.IC_pulse>(_motor->input_vars.IC_pulse_MID + _motor->input_vars.IC_pulse_DEADZONE)){
					  _motor->input_vars.RCPWM_req = (float)(_motor->input_vars.IC_pulse - (_motor->input_vars.IC_pulse_MID + _motor->input_vars.IC_pulse_DEADZONE))*_motor->input_vars.RCPWM_gain[0][1];
					  if(fabsf(_motor->input_vars.RCPWM_req>1.1f)){
						  handleError(_motor, ERROR_INPUT_OOR);
					  }
					  if(_motor->input_vars.RCPWM_req>1.0f){_motor->input_vars.RCPWM_req=1.0f;}
					  if(_motor->input_vars.RCPWM_req<-1.0f){_motor->input_vars.RCPWM_req=-1.0f;}
				  }
				  else if(_motor->input_vars.IC_pulse<(_motor->input_vars.IC_pulse_MID - _motor->input_vars.IC_pulse_DEADZONE)){
					  _motor->input_vars.RCPWM_req = ((float)_motor->input_vars.IC_pulse - (float)(_motor->input_vars.IC_pulse_MID - _motor->input_vars.IC_pulse_DEADZONE))*_motor->input_vars.RCPWM_gain[0][1];
					  if(fabsf(_motor->input_vars.RCPWM_req>1.1f)){
						  handleError(_motor, ERROR_INPUT_OOR);
					  }
					  if(_motor->input_vars.RCPWM_req>1.0f){_motor->input_vars.RCPWM_req=1.0f;}
					  if(_motor->input_vars.RCPWM_req<-1.0f){_motor->input_vars.RCPWM_req=-1.0f;}
				  } else{
					  _motor->input_vars.RCPWM_req = 0.0f;
				  }
			  }	else {//The duration of the IC was wrong; trap it and write no current request
				  //Todo maybe want to implement a timeout on this, allowing spurious pulses to not wiggle the current?
				  _motor->input_vars.RCPWM_req = 0.0f;
			  }
		  } else {//No pulse received flag
			  _motor->input_vars.RCPWM_req = 0.0f;
		  }
	  } else{
		  _motor->input_vars.RCPWM_req = 0.0f;
	  }

	  //ADC2 input
	  if(_motor->input_vars.input_options & 0b0010){
			  if(_motor->Raw.ADC_in_ext2>_motor->input_vars.adc2_MIN){
				  _motor->input_vars.ADC2_req = ((float)_motor->Raw.ADC_in_ext2-(float)_motor->input_vars.adc2_MIN)*_motor->input_vars.adc1_gain[1]*_motor->input_vars.ADC2_polarity;
				  if(_motor->Raw.ADC_in_ext2>_motor->input_vars.adc2_OOR){
					  //input_vars.ADC2_req = 0.0f;
					  handleError(_motor, ERROR_INPUT_OOR);
				  }
			  }
			  else{
				  _motor->input_vars.ADC2_req = 0.0f;
			  }
			  if(_motor->input_vars.ADC2_req>1.0f){_motor->input_vars.ADC2_req=1.0f;}
			  if(_motor->input_vars.ADC2_req<-1.0f){_motor->input_vars.ADC2_req=-1.0f;}
	  }else{
		  _motor->input_vars.ADC2_req = 0.0f;
	  }

	  //ADC1 input
	  if(_motor->input_vars.input_options & 0b0001){
			  if(_motor->Raw.ADC_in_ext1>_motor->input_vars.adc1_MIN){
				  _motor->input_vars.ADC1_req = ((float)_motor->Raw.ADC_in_ext1-(float)_motor->input_vars.adc1_MIN)*_motor->input_vars.adc1_gain[1]*_motor->input_vars.ADC1_polarity;
				  if(_motor->Raw.ADC_in_ext1>_motor->input_vars.adc1_OOR){
					  //input_vars.ADC1_req = 0.0f;//If we set throttle to zero, it will immediately reset the error!
					  handleError(_motor, ERROR_INPUT_OOR);
				  }
			  }
			  else{
				  _motor->input_vars.ADC1_req = 0.0f;
			  }
		  if(_motor->input_vars.ADC1_req>1.0f){_motor->input_vars.ADC1_req=1.0f;}
		  if(_motor->input_vars.ADC1_req<-1.0f){_motor->input_vars.ADC1_req=-1.0f;}
	  }else{
		  _motor->input_vars.ADC1_req = 0.0f;
	  }

#ifdef KILLSWITCH_GPIO
	  if(_motor->input_vars.input_options & 0b10000){//Killswitch
		if(KILLSWITCH_GPIO->IDR & (0x01<<KILLSWITCH_IONO)){
			_motor->input_vars.nKillswitch = 1;
			_motor->key_bits &= ~KILLSWITCH_KEY;
		}else{
			_motor->input_vars.nKillswitch = 0;
			_motor->key_bits |= KILLSWITCH_KEY;
		}
		if(_motor->input_vars.invert_killswitch){
			_motor->input_vars.nKillswitch = !_motor->input_vars.nKillswitch;
			_motor->key_bits ^= KILLSWITCH_KEY;
		}
	  }else{//If we are not using the killswitch, then it should be "on"
		  _motor->input_vars.nKillswitch = 1;
			_motor->key_bits &= ~KILLSWITCH_KEY;
	  }
#else
	  _motor->input_vars.nKillswitch = 1;
	  _motor->key_bits &= ~KILLSWITCH_KEY;
#endif
}

int handbrakenow;
int MESCinput_isHandbrake(){
#ifdef HANDBRAKE_GPIO
	handbrakenow = HANDBRAKE_GPIO->IDR & (0x01<<HANDBRAKE_IONO);
	if(HANDBRAKE_GPIO->IDR & (0x01<<HANDBRAKE_IONO)){
		return 1;
	}else{
		return 0;
	}
#else
return 0;
#endif
}
