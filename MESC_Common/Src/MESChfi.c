/*
 **
 ******************************************************************************
 * @file           : MESChfi.c
 * @brief          : HFI rotor position
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

#include "MESChfi.h"

static MESCiq_s Idq[2] = {{.d = 0.0f, .q = 0.0f}, {.d = 0.0f, .q = 0.0f}};
static volatile MESCiq_s dIdq = {.d = 0.0f, .q = 0.0f};
static volatile float magnitude45;
static MESCiq_s intdidq;

void MESChfi_Toggle(MESC_motor_typedef *_motor){
	if(((_motor->FOC.Vdq.q-_motor->FOC.Idq_smoothed.q*_motor->m.R) > _motor->HFI.toggle_voltage)||((_motor->FOC.Vdq.q-_motor->FOC.Idq_smoothed.q*_motor->m.R) < -_motor->HFI.toggle_voltage)||(_motor->MotorSensorMode==MOTOR_SENSOR_MODE_HALL)){
		_motor->HFI.inject = 0;
		_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH;
	} else if(((_motor->FOC.Vdq.q-_motor->FOC.Idq_smoothed.q*_motor->m.R) < (_motor->HFI.toggle_voltage-1.0f))&&((_motor->FOC.Vdq.q-_motor->FOC.Idq_smoothed.q*_motor->m.R) > -(_motor->HFI.toggle_voltage-1.0f)) &&(_motor->HFI.Type !=HFI_TYPE_NONE)){
		_motor->HFI.int_err = _motor->FOC.PLL_int;
		_motor->HFI.inject = 1;
		_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH*0.1f;
	}
}

void MESChfi_Run(MESC_motor_typedef *_motor){
  if ((_motor->HFI.inject)&&(_motor->MotorState != MOTOR_STATE_TRACKING)) {
	int Idqreq_dir=0;
	if (_motor->HFI.inject_high_low_now == 0){//First we create the toggle
		_motor->HFI.inject_high_low_now = 1;
		  Idq[0].d = _motor->FOC.Idq.d;
		  Idq[0].q = _motor->FOC.Idq.q;
	}else{
		_motor->HFI.inject_high_low_now = 0;
		  Idq[1].d = _motor->FOC.Idq.d;
		  Idq[1].q = _motor->FOC.Idq.q;
	  }
	_motor->FOC.didq.d = (Idq[0].d - Idq[1].d); //Calculate the changing current levels
	_motor->FOC.didq.q = (Idq[0].q - Idq[1].q);

	switch(_motor->HFI.Type){
		case HFI_TYPE_NONE:
		__NOP();
		break;
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		case HFI_TYPE_45:
			if(_motor->HFI.inject_high_low_now ==1){
				_motor->HFI.Vd_injectionV = +_motor->meas.hfi_voltage;
				if(_motor->FOC.Idq_req.q>0.0f){
					Idqreq_dir = 1;
					_motor->HFI.Vq_injectionV = +_motor->meas.hfi_voltage;
				}else{
					_motor->HFI.Vq_injectionV = -_motor->meas.hfi_voltage;
					Idqreq_dir = -1;
				}
			}else{
				_motor->HFI.Vd_injectionV = -_motor->meas.hfi_voltage;
				if(_motor->FOC.Idq_req.q>0.0f){
					_motor->HFI.Vq_injectionV = -_motor->meas.hfi_voltage;
				}else{
					_motor->HFI.Vq_injectionV = +_motor->meas.hfi_voltage;
				}
			}
			//Run the PLL
			magnitude45 = sqrtf(_motor->FOC.didq.d*_motor->FOC.didq.d+_motor->FOC.didq.q*_motor->FOC.didq.q);

			if(_motor->FOC.was_last_tracking==0){

				float error;
				//Estimate the angle error, the gain to be determined in the HFI detection and setup based on the HFI current and the max iteration allowable
				error = _motor->HFI.Gain*(magnitude45-_motor->HFI.mod_didq);
				if(error>500.0f){error = 500.0f;}
				if(error<-500.0f){error = -500.0f;}
				_motor->HFI.int_err = _motor->HFI.int_err +0.05f*error;
				if(_motor->HFI.int_err>1000.0f){_motor->HFI.int_err = 1000.0f;}
				if(_motor->HFI.int_err<-1000.0f){_motor->HFI.int_err = -1000.0f;}
				_motor->FOC.FOCAngle = _motor->FOC.FOCAngle + (int)(error + _motor->HFI.int_err)*Idqreq_dir;

			}else{
				_motor->FOC.FOCAngle += _motor->HFI.test_increment;
				_motor->HFI.accu += magnitude45;
				_motor->HFI.count += 1;
			}
			#if 0 //Sometimes for investigation we want to just lock the angle, this is an easy bodge
							_motor->FOC.FOCAngle = 62000;
			#endif
				break;
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		case HFI_TYPE_D:
			if(_motor->HFI.inject_high_low_now ==1){
			  _motor->HFI.Vd_injectionV = +_motor->meas.hfi_voltage;
			}else{
			  _motor->HFI.Vd_injectionV = -_motor->meas.hfi_voltage;
			}
			if(_motor->FOC.didq.q>1.0f){_motor->FOC.didq.q = 1.0f;}
			if(_motor->FOC.didq.q<-1.0f){_motor->FOC.didq.q = -1.0f;}
			intdidq.q = (intdidq.q + 0.1f*_motor->FOC.didq.q);
			if(intdidq.q>10){intdidq.q=10;}
			if(intdidq.q<-10){intdidq.q=-10;}
			_motor->FOC.FOCAngle += (int)(250.0f*_motor->FOC.IIR[1] + 10.50f*intdidq.q)*_motor->FOC.d_polarity;
		break;
		case HFI_TYPE_SPECIAL:
			__NOP();
			if(_motor->HFI.inject_high_low_now ==1){
			  _motor->HFI.Vd_injectionV = _motor->HFI.special_injectionVd;
			  _motor->HFI.Vq_injectionV = _motor->HFI.special_injectionVq;
			}else{
				_motor->HFI.Vd_injectionV = -_motor->HFI.special_injectionVd;
				_motor->HFI.Vq_injectionV = -_motor->HFI.special_injectionVq;			}
		break;
	}
  }else {
	  _motor->HFI.Vd_injectionV = 0.0f;
	  _motor->HFI.Vq_injectionV = 0.0f;
  }
}

void MESChfi_Slow(MESC_motor_typedef *_motor){
	/////////////Set and reset the HFI////////////////////////
		switch(_motor->HFI.Type){
			case HFI_TYPE_45:
			MESChfi_Toggle(_motor);
				if(_motor->HFI.inject==1){
					//static int no_q;
					if(_motor->FOC.was_last_tracking==1){
						if(_motor->HFI.countdown>1){
							_motor->HFI.mod_didq = _motor->HFI.accu / _motor->HFI.count;
							_motor->HFI.Gain = 5000.0f/_motor->HFI.mod_didq;
							_motor->FOC.was_last_tracking = 0;
						}else{
							_motor->HFI.test_increment = 65536 * SLOW_LOOP_FREQUENCY / _motor->FOC.pwm_frequency;
							_motor->HFI.countdown++;
						}
					}else{
						_motor->HFI.countdown = 0;
						_motor->HFI.count = 0;
						_motor->HFI.accu = 0.0f;
					}
				}
			break;

			case HFI_TYPE_D:
				MESChfi_Toggle(_motor);
				if(_motor->HFI.inject==1){
					if(_motor->HFI.countdown==3){
						_motor->FOC.Idq_req.d = HFI_TEST_CURRENT;
						_motor->FOC.Idq_req.q=0.0f;//Override the inputs to set Q current to zero
					}else if(_motor->HFI.countdown==2){
						_motor->FOC.Ldq_now_dboost[0] = _motor->FOC.IIR[0]; //Find the effect of d-axis current
						_motor->FOC.Idq_req.d = 1.0f;
						_motor->FOC.Idq_req.q=0.0f;
					}else if(_motor->HFI.countdown == 1){
						_motor->FOC.Idq_req.d = -HFI_TEST_CURRENT;
						_motor->FOC.Idq_req.q=0.0f;
					}else if(_motor->HFI.countdown == 0){
						_motor->FOC.Ldq_now[0] = _motor->FOC.IIR[0];//_motor->HFI.Vd_injectionV;
						_motor->FOC.Idq_req.d = 0.0f;
					if(_motor->FOC.Ldq_now[0]>_motor->FOC.Ldq_now_dboost[0]){_motor->FOC.FOCAngle+=32768;}
					_motor->HFI.countdown = 200;
					}
					_motor->HFI.countdown--;
				}
			break;

			case HFI_TYPE_NONE:
				_motor->HFI.inject = 0;
//				_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH;
			break;
			case HFI_TYPE_SPECIAL:
				MESChfi_Toggle(_motor);
			break;
		}
}
