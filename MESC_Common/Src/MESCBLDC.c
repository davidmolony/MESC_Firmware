/*
 **
 ******************************************************************************
 * @file           : MESCBLDC.c
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

 * MESCBLDC.c
 *
 *  Created on: 25 Jul 2020
 *      Author: David Molony
 */

#include "MESCBLDC.h"

#include "MESCfoc.h"
#include "MESCpwm.h"
#include "MESChw_setup.h"
#include "MESCmotor_state.h"

foc_measurement_t measurement_buffers;

extern TIM_HandleTypeDef htim1;

MESCBLDCVars_s BLDCVars;
MESCBLDCState_e BLDCState;

void BLDCInit() {
  BLDCVars.ReqCurrent = 0;  // Start the motor at 0 current
  BLDCVars.BLDCduty = 0;
  BLDCVars.CurrentChannel = 0;
  BLDCVars.currentCurrent = 0;
  BLDCVars.pGain =
      1023 * mtr[0].m.R /
      8;  // wtf should I set the gain as by default... V/Amp error...Perhaps
          // base it on Rphase and the bus voltage (nominally 48V)? But we don;t
          // know the exact bus voltage yet...
  BLDCVars.iGain = BLDCVars.pGain;  // After experimentation, igain of pgain
                                    // seems to work well.
  BLDCVars.BLDCEstate = GetHallState();
  BLDCState = BLDC_FORWARDS;
}

void BLDCCommuteHall() {
  int CurrentHallState =
      GetHallState();  // Borrow the hall state detection from the FOC system
  static int LastHallState = 7;

  if (BLDCState == BLDC_FORWARDS) {
    BLDCVars.BLDCEstate = (CurrentHallState + 2) % 6;
    writeBLDC();  // Write the PWM values for the next state to generate
                  // forward torque
    if (!(BLDCVars.BLDCEstate == (CurrentHallState + 1))) {
      // ToDo Fix if the writeBLDC command is put in here, the PWM duty
      // gets stuck at 0.
    }
  } else if (BLDCState == BLDC_BACKWARDS) {
    BLDCVars.BLDCEstate = (CurrentHallState + 4) % 6;
    writeBLDC();  // Write the PWM values for the previous state to generate
                  // reverse torque
                  // FIXME: what is this supposed to accomplish?
    // commented out since this code does nothing and is likely removed by
    // the compiler.     if(!(CurrentHallState==CurrentHallState)){
    //    }
  } else if (BLDCState == BLDC_BRAKE) {
    int hallStateChange = CurrentHallState - LastHallState;
    // ToDo Logic to always be on synch or hanging 1 step in front or
    // behind... ToDo this does not cope with the roll-over, making for a
    // very jerky brake
    // TODO: the expression inside if() statement is very hard to read.
    // Create separate variable.
    if (((hallStateChange) % 6) > 1) {
      BLDCVars.BLDCEstate = (CurrentHallState + 5) % 6;
    } else if (((CurrentHallState - LastHallState) % 6) < -1) {
      BLDCVars.BLDCEstate = (CurrentHallState + 1) % 6;
      LastHallState = CurrentHallState;
    }
    writeBLDC();
  } else {
    // Disable the drivers, freewheel
    // fixme: misleading function name. If this is freewheel, then it should
    // be named as such.
    MESCpwm_phU_Break(&mtr[0]);
    MESCpwm_phV_Break(&mtr[0]);
    MESCpwm_phW_Break(&mtr[0]);
  }
}

void BLDCCurrentController() {
  // Implement a simple PI controller
  static float CurrentError = 0;
  static float CurrentIntegralError = 0;
  static int Duty = 0;

  BLDCVars.currentCurrent =
      measurement_buffers.ConvertedADC[BLDCVars.CurrentChannel][0];

  CurrentError = (BLDCVars.ReqCurrent - BLDCVars.currentCurrent);
  // measurement_buffers.ConvertedADC[BLDCVars.CurrentChannel][0]);

  CurrentIntegralError =
      CurrentIntegralError +
      CurrentError *
          0.0027f;  // 37kHz PWM, so the integral portion should
                   // be multiplied by 1/37k before accumulating Interesting
                   // behaviour with gating - increasing the factor here and
                   // changing the pgain have different results -  need to have
                   // high gain prior to gating and stability becomes much
                   // better.

  if (CurrentIntegralError > 20) CurrentIntegralError = 20;    // Magic numbers
  if (CurrentIntegralError < -20) CurrentIntegralError = -20;  // Magic numbers

  Duty = (int)(CurrentError * BLDCVars.pGain +
               CurrentIntegralError * BLDCVars.iGain);

  if (Duty > 1023) {
    Duty = 1023;
  } else if (Duty < 0) {
    Duty = 0;
  }
  BLDCVars.BLDCduty = Duty;
}

void writeBLDC() {
  switch (BLDCVars.BLDCEstate) {
    case 0:
      // disable phase first
      MESCpwm_phW_Break(&mtr[0]);
      // WritePWM values
      htim1.Instance->CCR1 = BLDCVars.BLDCduty;
      htim1.Instance->CCR2 = 0;
      MESCpwm_phU_Enable(&mtr[0]);
      MESCpwm_phV_Enable(&mtr[0]);
      BLDCVars.CurrentChannel =
          1;  // Write the field into which the lowside current will flow,
              // to be retrieved from the FOC_measurement_vars
      break;

    case 1:
      MESCpwm_phV_Break(&mtr[0]);
      htim1.Instance->CCR1 = BLDCVars.BLDCduty;
      htim1.Instance->CCR3 = 0;
      MESCpwm_phU_Enable(&mtr[0]);
      MESCpwm_phW_Enable(&mtr[0]);
      BLDCVars.CurrentChannel = 2;
      break;

    case 2:
      MESCpwm_phU_Break(&mtr[0]);
      htim1.Instance->CCR2 = BLDCVars.BLDCduty;
      htim1.Instance->CCR3 = 0;
      MESCpwm_phV_Enable(&mtr[0]);
      MESCpwm_phW_Enable(&mtr[0]);
      BLDCVars.CurrentChannel = 2;
      break;

    case 3:
      MESCpwm_phW_Break(&mtr[0]);
      htim1.Instance->CCR1 = 0;
      htim1.Instance->CCR2 = BLDCVars.BLDCduty;
      MESCpwm_phU_Enable(&mtr[0]);
      MESCpwm_phV_Enable(&mtr[0]);
      BLDCVars.CurrentChannel = 0;
      break;

    case 4:
      MESCpwm_phV_Break(&mtr[0]);
      htim1.Instance->CCR1 = 0;
      htim1.Instance->CCR3 = BLDCVars.BLDCduty;
      MESCpwm_phU_Enable(&mtr[0]);
      MESCpwm_phW_Enable(&mtr[0]);
      BLDCVars.CurrentChannel = 0;
      break;

    case 5:
      MESCpwm_phU_Break(&mtr[0]);
      htim1.Instance->CCR2 = 0;
      htim1.Instance->CCR3 = BLDCVars.BLDCduty;
      MESCpwm_phV_Enable(&mtr[0]);
      MESCpwm_phW_Enable(&mtr[0]);
      BLDCVars.CurrentChannel = 1;
      break;
    default:
      break;
  }
}

int GetHallState() {
  switch (getHallState()) {
    case 0:
      return 7;  // 7 is the no hall sensor detected state (all low)
      break;
    case 7:
      return 6;  // 6 is the no hall sensor detected state (all high)
      break;
      // Implement the hall table order here, depending how the hall
      // sensors are configured
    case 1:
      return 0;
      break;
    case 3:
      return 1;
      break;
    case 2:
      return 2;
      break;
    case 6:
      return 3;
      break;
    case 4:
      return 4;
      break;
    case 5:
      return 5;
      break;
    default:
      return 8;
      break;
  }
}

void BLDCCommute(MESC_motor_typedef *_motor){

//Collect the variables required
	switch (_motor->BLDC.sector){
		case 0:
		_motor->BLDC.I_meas = _motor->Conv.Iu;
//		_motor->BLDC.V_meas = _motor->Conv.Vv;
//		_motor->BLDC.rising_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
		break;

		case 1:
			_motor->BLDC.I_meas = _motor->Conv.Iu;
//			_motor->BLDC.V_meas = _motor->Conv.Vw;
//			_motor->BLDC.falling_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
		break;

		case 2:
			_motor->BLDC.I_meas = _motor->Conv.Iw;
//			_motor->BLDC.V_meas = _motor->Conv.Vu;
//			_motor->BLDC.rising_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
		break;

		case 3:
			_motor->BLDC.I_meas = _motor->Conv.Iw;
//			_motor->BLDC.V_meas = _motor->Conv.Vv;
//			_motor->BLDC.falling_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
		break;

		case 4:
			_motor->BLDC.I_meas = _motor->Conv.Iv;
//			_motor->BLDC.V_meas = _motor->Conv.Vw;
//			_motor->BLDC.rising_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
		break;

		case 5:
			_motor->BLDC.I_meas = _motor->Conv.Iv;
//			_motor->BLDC.V_meas = _motor->Conv.Vu;
//			_motor->BLDC.falling_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
		break;
	}
//	//Reduce the rising and falling integrals
//	_motor->BLDC.rising_int = _motor->BLDC.rising_int * 0.999f;
//	_motor->BLDC.falling_int = _motor->BLDC.rising_int * 0.999f;

	//Invert the current since we are measuring the current leaving the motor but controlling the voltage where current is going in
	_motor->BLDC.I_meas = -_motor->BLDC.I_meas;

//////Run PID
	_motor->BLDC.I_pgain = _motor->FOC.Iq_pgain;//Borrow from FOC for now
	_motor->BLDC.I_igain = _motor->FOC.Iq_igain;//Borrow from FOC for now
	_motor->BLDC.PWM_period = _motor->FOC.pwm_period;//Borrow from FOC for now

	//Calculate the error
	_motor->BLDC.I_error = (_motor->BLDC.I_set-_motor->BLDC.I_meas)*_motor->BLDC.I_pgain;
	_motor->BLDC.int_I_error = //Calculate the integral error
			_motor->BLDC.int_I_error + _motor->BLDC.I_error * _motor->BLDC.I_igain * _motor->BLDC.PWM_period;

	_motor->BLDC.V_bldc = _motor->BLDC.int_I_error + _motor->BLDC.I_error;
	//Bounding
	if(_motor->BLDC.V_bldc > 0.95f * _motor->Conv.Vbus){
		_motor->BLDC.V_bldc = 0.95f * _motor->Conv.Vbus;
		if(_motor->BLDC.int_I_error > _motor->Conv.Vbus){
			_motor->BLDC.int_I_error = _motor->Conv.Vbus;
			_motor->BLDC.I_error = 0.05f*_motor->BLDC.int_I_error;
		}
	}
	//Determine the conversion from volts to PWM
	_motor->BLDC.V_bldc_to_PWM = _motor->mtimer->Instance->ARR/_motor->Conv.Vbus;
	//Convert to PWM value
	_motor->BLDC.BLDC_PWM = _motor->BLDC.V_bldc*_motor->BLDC.V_bldc_to_PWM;


//////Integrate and determine if commutation ready, VBEMF = Vbldc-2*I*Rphase
	_motor->BLDC.flux_integral = _motor->BLDC.flux_integral + (_motor->BLDC.V_bldc - _motor->BLDC.I_meas * 2.0f*_motor->m.R)* _motor->BLDC.PWM_period; //Volt seconds

	//FUDGED
	_motor->BLDC.closed_loop = 1;
	if(_motor->BLDC.closed_loop){
		//If the flux reaches a limit  then commute
		if(_motor->BLDC.flux_integral<0.0f){_motor->BLDC.flux_integral = 0.0f;}
		if(_motor->BLDC.flux_integral>_motor->BLDC.com_flux){
			_motor->BLDC.V_meas_sect[_motor->BLDC.sector] = _motor->BLDC.V_meas;

			_motor->BLDC.sector = _motor->BLDC.sector + _motor->BLDC.direction;
			_motor->BLDC.last_flux_integral = _motor->BLDC.flux_integral;
			_motor->BLDC.flux_integral = 0.0f; //Reset the integrator
			_motor->BLDC.last_p_error = _motor->BLDC.I_error;
			//Run a vague tuning mechanism, needs a lot of work.
			if((_motor->BLDC.int_I_error>_motor->Conv.Vbus*0.4f) && (_motor->BLDC.int_I_error<_motor->Conv.Vbus*0.9f)){
				if(_motor->BLDC.I_error>0.05f*_motor->BLDC.int_I_error){
					_motor->BLDC.com_flux = _motor->BLDC.com_flux*1.005f;
				}
				if(_motor->BLDC.I_error<0.05f*_motor->BLDC.int_I_error){
					_motor->BLDC.com_flux = _motor->BLDC.com_flux*0.99f;
				}
			}

//			_motor->BLDC.rising_int_st =_motor->BLDC.rising_int;
//			_motor->BLDC.rising_int = 0.0f;
//			_motor->BLDC.falling_int_st = _motor->BLDC.falling_int;
//			_motor->BLDC.falling_int = 0.0f;
//			if(_motor->BLDC.falling_int_st > _motor->BLDC.falling_int_st){
//				_motor->BLDC.com_flux = _motor->BLDC.com_flux * 1.01f;
//			}else{
//				_motor->BLDC.com_flux = _motor->BLDC.com_flux * 0.99f;
//			}
//
//			if(_motor->BLDC.com_flux<0.018f){_motor->BLDC.com_flux=0.018f;}
//			if(_motor->BLDC.com_flux>0.022f){_motor->BLDC.com_flux=0.022f;}

		}
	}else{
		_motor->BLDC.OL_countdown--;
		if(_motor->BLDC.OL_countdown == 0){
			_motor->BLDC.OL_countdown =_motor->BLDC.OL_periods;
			_motor->BLDC.sector++;
			_motor->BLDC.last_flux_integral = _motor->BLDC.flux_integral;
			_motor->BLDC.flux_integral = 0.0f; //Reset the integrator
		}
	}

//////Wrap the sector
	if(_motor->BLDC.sector>5){
		_motor->BLDC.sector = 0;
	}else if(_motor->BLDC.sector<0){
		_motor->BLDC.sector = 5;
	}

//////Write PWMs
	switch (_motor->BLDC.sector){
		case 0:
			MESCpwm_phV_Break(_motor);
			MESCpwm_phU_Enable(_motor);
			MESCpwm_phW_Enable(_motor);
			_motor->mtimer->Instance->CCR1 = 0;
			_motor->mtimer->Instance->CCR3 = _motor->BLDC.BLDC_PWM;
		break;
		case 1:
			MESCpwm_phW_Break(_motor);
			MESCpwm_phU_Enable(_motor);
			MESCpwm_phV_Enable(_motor);
			_motor->mtimer->Instance->CCR1 = 0;
			_motor->mtimer->Instance->CCR2 = _motor->BLDC.BLDC_PWM;
		break;
		case 2:
			MESCpwm_phU_Break(_motor);
			MESCpwm_phV_Enable(_motor);
			MESCpwm_phW_Enable(_motor);
			_motor->mtimer->Instance->CCR3 = 0;
			_motor->mtimer->Instance->CCR2 = _motor->BLDC.BLDC_PWM;
		break;
		case 3:
			MESCpwm_phV_Break(_motor);
			MESCpwm_phU_Enable(_motor);
			MESCpwm_phW_Enable(_motor);
			_motor->mtimer->Instance->CCR3 = 0;
			_motor->mtimer->Instance->CCR1 = _motor->BLDC.BLDC_PWM;
		break;
		case 4:
			MESCpwm_phW_Break(_motor);
			MESCpwm_phU_Enable(_motor);
			MESCpwm_phV_Enable(_motor);
			_motor->mtimer->Instance->CCR2 = 0;
			_motor->mtimer->Instance->CCR1 = _motor->BLDC.BLDC_PWM;
		break;
		case 5:
			MESCpwm_phU_Break(_motor);
			MESCpwm_phV_Enable(_motor);
			MESCpwm_phW_Enable(_motor);
			_motor->mtimer->Instance->CCR2 = 0;
			_motor->mtimer->Instance->CCR3 = _motor->BLDC.BLDC_PWM;
		break;
		default:
			//Reset to 0, something went wrong...
			_motor->BLDC.sector = 0;
		break;

	}
}

void CalculateBLDCGains(MESC_motor_typedef *_motor){

}
