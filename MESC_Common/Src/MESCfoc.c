/*
 **
 ******************************************************************************
 * @file           : MESCfoc.c
 * @brief          : FOC running code and ADC buffers
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 David Molony.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
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

 * MESCfoc.c
 *
 *  Created on: 18 Jul 2020
 *      Author: David Molony
 */

/* Includes ------------------------------------------------------------------*/
#include "MESCfoc.h"

#include "MESCBLDC.h"
#include "MESChw_setup.h"
#include "MESCmotor_state.h"
#include "MESCsin_lut.h"
#include "MESCmotor.h"
#include "MESCtemp.h"
#include "MESCspeed.h"
#include "MESCerror.h"

#include <math.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1, hadc2, hadc3, hadc4;

float one_on_sqrt6 = 0.408248f;
float one_on_sqrt3 = 0.577350f;
float one_on_sqrt2 = 0.707107f;
float sqrt_two_on_3 = 0.816497f;
float sqrt3_2 = 1.22474f;
float sqrt2 = 1.41421f;
float sqrt1_2 = 0.707107f;
float sqrt3_on_2 = 0.866025f;
float two_on_sqrt3 = 1.73205f;
int adc_conv_end;
//static float flux_linked_alpha = 0.00501f;
//static float flux_linked_beta = 0.00511f;

MESC_motor_typedef motor1;
MESC_motor_typedef motor2;


MESCfoc_s foc_var;
MESCtest_s test_vals;
foc_measurement_t measurement_buffers;
input_vars_t input_vars;
sampled_vars_t sampled_vars;

int print_samples_now, lognow;

void MESCInit(MESC_motor_typedef *_motor) {
#ifdef STM32L4 // For some reason, ST have decided to have a different name for the L4 timer DBG freeze...
	DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP;
#else
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
#endif
	MotorState = MOTOR_STATE_IDLE;




  mesc_init_1(_motor);

  HAL_Delay(3000);  // Give the everything else time to start up (e.g. throttle,
                    // controller, PWM source...)

  mesc_init_2(_motor);

  hw_init(_motor);  // Populate the resistances, gains etc of the PCB - edit within
              // this function if compiling for other PCBs


  // Start the PWM channels, reset the counter to zero each time to avoid
  // triggering the ADC, which in turn triggers the ISR routine and wrecks the
  // startup
  mesc_init_3(_motor);
  MotorState = MOTOR_STATE_INITIALISING;
#ifdef LOGGING
  lognow = 1;
#endif

#ifdef USE_ENCODER
  _motor->FOC.enc_offset = ENCODER_E_OFFSET;
#endif

	//Set up the input capture for throttle
	HAL_TIM_IC_Start(_motor->stimer, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(_motor->stimer, TIM_CHANNEL_2);
	// Here we can auto set the prescaler to get the us input regardless of the main clock
	__HAL_TIM_SET_PRESCALER(_motor->stimer, (HAL_RCC_GetHCLKFreq() / 1000000 - 1));
	__HAL_TIM_ENABLE_IT(_motor->stimer, TIM_IT_UPDATE);

  InputInit();

  	  //htim1.Instance->BDTR |=TIM_BDTR_MOE;
	  // initialising the comparators triggers the break state,
	  // so turn it back on
	  // At this point we just let the whole thing run off into interrupt land, and
	  // the fastLoop() starts to be triggered by the ADC conversion complete
	  // interrupt

	motor.Rphase = motor_profile->R;
	motor.Lphase = motor_profile->L_D;
	motor.Lqphase = motor_profile->L_Q;
	motor.motor_flux = motor_profile->flux_linkage;
	motor.uncertainty = 1;

	calculateGains(_motor);
	calculateVoltageGain(_motor);

}

void InputInit(){

	input_vars.max_request_Idq.d = 0.0f; //Not supporting d-axis input current for now
	input_vars.min_request_Idq.d = 0.0f;
	input_vars.max_request_Idq.q = MAX_IQ_REQUEST;
	input_vars.min_request_Idq.q = -MAX_IQ_REQUEST;

	input_vars.IC_pulse_MAX = IC_PULSE_MAX;
	input_vars.IC_pulse_MIN = IC_PULSE_MIN;
	input_vars.IC_pulse_MID = IC_PULSE_MID;
	input_vars.IC_pulse_DEADZONE = IC_PULSE_DEADZONE;
	input_vars.IC_duration_MAX = IC_DURATION_MAX;
	input_vars.IC_duration_MIN = IC_DURATION_MIN;


	input_vars.adc1_MAX = ADC1MAX;
	input_vars.adc1_MIN = ADC1MIN;

	input_vars.adc2_MAX = ADC2MAX;
	input_vars.adc2_MIN = ADC2MIN;

	input_vars.adc1_gain[0] = (input_vars.max_request_Idq.d)/(input_vars.adc1_MAX-input_vars.adc1_MIN);
	input_vars.adc1_gain[1] = (input_vars.max_request_Idq.q)/(input_vars.adc1_MAX-input_vars.adc1_MIN);

	input_vars.adc2_gain[0] = (input_vars.max_request_Idq.d)/(input_vars.adc2_MAX-input_vars.adc2_MIN);
	input_vars.adc2_gain[1] = (input_vars.max_request_Idq.q)/(input_vars.adc2_MAX-input_vars.adc2_MIN);

	//RCPWM forward gain//index [0][x] is used for Idq requests for now, might support asymmetric brake and throttle later
	input_vars.RCPWM_gain[0][0] = (input_vars.max_request_Idq.d)/((float)input_vars.IC_pulse_MAX - (float)input_vars.IC_pulse_MID - (float)input_vars.IC_pulse_DEADZONE);
	input_vars.RCPWM_gain[0][1] = (input_vars.max_request_Idq.q)/(((float)input_vars.IC_pulse_MID - (float)input_vars.IC_pulse_DEADZONE)-(float)input_vars.IC_pulse_MIN);

	input_vars.input_options = DEFAULT_INPUT;
	input_vars.ADC1_polarity = ADC1_POLARITY;
	input_vars.ADC2_polarity = ADC2_POLARITY;

	input_vars.Idq_req_UART.d =0;
	input_vars.Idq_req_RCPWM.d =0;
	input_vars.Idq_req_ADC1.d =0;
	input_vars.Idq_req_ADC2.d =0;
	input_vars.Idq_req_UART.q =0;
	input_vars.Idq_req_RCPWM.q =0;
	input_vars.Idq_req_ADC1.q =0;
	input_vars.Idq_req_ADC2.q =0;

}
void initialiseInverter(MESC_motor_typedef *_motor){

      _motor->offset.Iu += _motor->Raw.Iu;
      _motor->offset.Iv += _motor->Raw.Iv;
      _motor->offset.Iw += _motor->Raw.Iw;


      static int initcycles = 0;
      initcycles = initcycles + 1;
      //Exit the initialisation after 1000cycles
      if (initcycles == 1000) {
        calculateGains(_motor);
        calculateVoltageGain(_motor);
        _motor->FOC.flux_linked_beta = 0.001f;
        _motor->FOC.flux_linked_alpha = 0.001f;

        _motor->offset.Iu /=  initcycles;
        _motor->offset.Iv /=  initcycles;
        _motor->offset.Iw /=  initcycles;
//ToDo, do we want some safety checks here like offsets being roughly correct?
    	MotorState = MOTOR_STATE_TRACKING;
        htim1.Instance->BDTR |= TIM_BDTR_MOE;
      }
}


// This should be the only function needed to be added into the PWM interrupt
// for MESC to run Ensure that it is followed by the clear timer update
// interrupt
void MESC_PWM_IRQ_handler(MESC_motor_typedef *_motor) {
  if (_motor->mtimer->Instance->CNT > 512) {
    //_motor->FOC.IRQentry = debugtim.Instance->CNT;
    fastLoop(_motor);
    //_motor->FOC.IRQexit = debugtim.Instance->CNT - _motor->FOC.IRQentry;
    //_motor->FOC.FLrun++;
  } else {
    //_motor->FOC.IRQentry = debugtim.Instance->CNT;
    hyperLoop(_motor);
    //_motor->FOC.IRQexit = debugtim.Instance->CNT - _motor->FOC.IRQentry;
    //_motor->FOC.VFLrun++;
  }
}

// The fastloop runs at PWM timer counter top, which is when the new ADC current
// readings arrive.
// The first few clock cycles of the interrupt should not use the adc readings,
// since the currents require approximately 1us = 144 clock cycles (f405) and 72
// clock cycles (f303) to convert.

static int current_hall_state;

void fastLoop(MESC_motor_typedef *_motor) {
  // Call this directly from the TIM top IRQ
  current_hall_state = getHallState(); //ToDo, this macro is not applicable to dual motors
  // First thing we ever want to do is convert the ADC values
  // to real, useable numbers.
  ADCConversion(_motor);
  adc_conv_end = htim1.Instance->CNT;  // track the ADC conversion time

  switch (MotorState) {

  	case MOTOR_STATE_INITIALISING:
  		initialiseInverter(_motor);
  		break;

    case MOTOR_STATE_RUN:
      // transform
      //      if (MotorControlType ==
      //          MOTOR_CONTROL_TYPE_BLDC) {
    	// BLDC is hopefully just a
      //          temporary "Get it spinning" kind of thing, to be deprecated in favour of FOC
      //        BLDCCurrentController();
      //        BLDCCommuteHall();
      //      }//For now we are going to not support BLDC mode
    	generateEnable(_motor);
      if (MotorSensorMode == MOTOR_SENSOR_MODE_HALL) {
			_motor->FOC.inject = 0;
			hallAngleEstimator();
			angleObserver(_motor);
			MESCFOC(_motor);
			writePWM(_motor);
      } else if (MotorSensorMode == MOTOR_SENSOR_MODE_SENSORLESS) {
#ifdef USE_HALL_START
    static int hall_start_now;
		if((fabs(_motor->FOC.Vdq.q-motor.Rphase*_motor->FOC.Idq_smoothed.q)<HALL_VOLTAGE_THRESHOLD)&&(_motor->FOC.hall_initialised)&&(current_hall_state>0)&&(current_hall_state<7)){
				hall_start_now = 1;
		}else if(fabs(_motor->FOC.Vdq.q-motor.Rphase*_motor->FOC.Idq_smoothed.q)>HALL_VOLTAGE_THRESHOLD+2.0f){
			hall_start_now = 0;
		}
		if(hall_start_now){
			_motor->FOC.flux_linked_alpha = 0.01f*_motor->FOC.flux_linked_alpha + 0.99f*_motor->FOC.hall_flux[current_hall_state-1][0];
			_motor->FOC.flux_linked_beta = 0.01f*_motor->FOC.flux_linked_beta + 0.99f*_motor->FOC.hall_flux[current_hall_state-1][1];
			_motor->FOC.FOCAngle = (uint16_t)(32768.0f + 10430.0f * fast_atan2(_motor->FOC.flux_linked_beta, _motor->FOC.flux_linked_alpha)) - 32768;
		}else{
			flux_observer(_motor);
		}
#else
    	flux_observer();
#endif
			MESCFOC(_motor);
			writePWM(_motor);
      } else if (MotorSensorMode == MOTOR_SENSOR_MODE_ENCODER) {
		  _motor->FOC.FOCAngle = _motor->FOC.enc_angle;
		  MESCFOC(_motor);
		  writePWM(_motor);
      } else if(MotorSensorMode == MOTOR_SENSOR_MODE_OPENLOOP){
		  _motor->FOC.openloop_step = 60;
		  OLGenerateAngle(_motor);
		  MESCFOC(_motor);
		  writePWM(_motor);
      }
      break;

    case MOTOR_STATE_TRACKING:
#ifdef HAS_PHASE_SENSORS
			  // Track using BEMF from phase sensors
			  generateBreak(_motor);
			  ADCPhaseConversion(_motor);
		      MESCTrack(_motor);
		      if (MotorSensorMode == MOTOR_SENSOR_MODE_HALL) {
		          hallAngleEstimator(_motor);
		          angleObserver(_motor);
		      } else if (MotorSensorMode == MOTOR_SENSOR_MODE_SENSORLESS) {
		    	  flux_observer(_motor);
#ifdef USE_HALL_START
		    	  HallFluxMonitor(_motor);
#endif
		      } else if (MotorSensorMode == MOTOR_SENSOR_MODE_ENCODER) {
					_motor->FOC.FOCAngle = _motor->FOC.enc_angle;
		      }
#endif

      break;

    case MOTOR_STATE_OPEN_LOOP_STARTUP:
      // Same as open loop
    	_motor->FOC.openloop_step = 60;
    	OLGenerateAngle(_motor);
    	MESCFOC(_motor);
    	writePWM(_motor);
      // Write the PWM values
      break;

    case MOTOR_STATE_OPEN_LOOP_TRANSITION:
      // Run open loop
      // Run observer
      // RunFOC
      // Weighted average of the outputs N PWM cycles
      // Write the PWM values
      break;

    case MOTOR_STATE_IDLE:
        generateBreak(_motor);
      // Do basically nothing
      break;

    case MOTOR_STATE_DETECTING:

      if ((current_hall_state == 7)) { // no hall sensors detected, all GPIO pulled high
        MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
        MotorState = MOTOR_STATE_GET_KV;
      } else if (current_hall_state == 0) {
        MotorState = MOTOR_STATE_ERROR;
        MotorError = MOTOR_ERROR_HALL0;
      } else {
        // hall sensors detected
        MotorSensorMode = MOTOR_SENSOR_MODE_HALL;
        getHallTable(_motor);
        MESCFOC(_motor);
        writePWM(_motor);
      }
      break;

    case MOTOR_STATE_MEASURING:
    			// Every PWM cycle we enter this function until
                // the resistance measurement has converged at a
                // good value. Once the measurement is complete,
                // Rphase is set, and this is no longer called
          measureResistance(_motor);
        break;

    case MOTOR_STATE_GET_KV:
      getkV(_motor);

      break;

    case MOTOR_STATE_ERROR:
      generateBreak(_motor);  // Generate a break state (software disabling all PWM)
                        // Now panic and freak out
      break;

    case MOTOR_STATE_ALIGN:
      // Turn on at a given voltage at electricalangle0;
      break;

    case MOTOR_STATE_TEST:
    	if(TestMode == TEST_TYPE_DOUBLE_PULSE){
      // Double pulse test
      doublePulseTest(_motor);
    	}else if(TestMode == TEST_TYPE_DEAD_TIME_IDENT){
    	//Here we are going to pull all phases low, and then increase the duty on one phase until we register a current response.
    	//This duty represents the dead time during which there is no current response
    		getDeadtime(_motor);
    	}else if(TestMode == TEST_TYPE_HARDWARE_VERIFICATION){
    		//Here we want a function that pulls all phases low, then all high and verifies a response
    		//Then we want to show a current response with increasing phase duty
    	}
      break;

    case MOTOR_STATE_RECOVERING:
	      deadshort(_motor); //Function to startup motor from running without phase sensors
      break;

    case MOTOR_STATE_SLAMBRAKE:
      if((fabs(_motor->Conv.Iu)>input_vars.max_request_Idq.q)||
		  (fabs(_motor->Conv.Iv)>input_vars.max_request_Idq.q)||
		  (fabs(_motor->Conv.Iw)>input_vars.max_request_Idq.q)){
    	  generateBreak(_motor);
      }else{
    	  generateEnable(_motor);
    	  htim1.Instance->CCR1 = 0;
    	  htim1.Instance->CCR2 = 0;
    	  htim1.Instance->CCR3 = 0;
    	  //We use "0", since this corresponds to all high side FETs off, always, and all low side ones on, always.
    	  //This means that current measurement can continue on low side and phase shunts, so over current protection remains active.
      }
    break;

    default:
      MotorState = MOTOR_STATE_ERROR;
      generateBreak(_motor);
      break;
  }
#ifdef SOFTWARE_ADC_REGULAR
       HAL_ADC_Start(&hadc1); //Try to eliminate the HAL call, slow and inefficient. Leaving this here for now.
        //hadc1.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
#endif
}

// The hyperloop runs at PWM timer bottom, when the PWM is in V7 (all high)
// In this loop, we write the values of the PWM to be updated at the next update
// event (timer top) This is where we want to inject signals for measurement so
// that the next signal level takes affect right after the ADC reading In normal
// run mode, we want to increment the angle and write the next PWM values
static MESCiq_s Idq[2] = {{.d = 0.0f, .q = 0.0f}, {.d = 0.0f, .q = 0.0f}};
static MESCiq_s dIdq = {.d = 0.0f, .q = 0.0f};
//static float IIR[2] = {0.0f, 0.0f};
static MESCiq_s intdidq;
static volatile float nrm;
static volatile float nrm_avg;
static uint16_t last_angle;

void hyperLoop(MESC_motor_typedef *_motor) {
//#ifdef USE_HFI
  if (_motor->FOC.inject) {
    if (_motor->FOC.inject_high_low_now == 0) {
      _motor->FOC.inject_high_low_now = 1;
      _motor->FOC.Vdq.d = _motor->FOC.Vdq.d + _motor->FOC.Vd_injectionV;
      _motor->FOC.Vdq.q = _motor->FOC.Vdq.q + _motor->FOC.Vq_injectionV;
      Idq[0].d = _motor->FOC.Idq.d;
      Idq[0].q = _motor->FOC.Idq.q;
    } else if (_motor->FOC.inject_high_low_now == 1) {
      _motor->FOC.inject_high_low_now = 0;
      _motor->FOC.Vdq.d = _motor->FOC.Vdq.d - _motor->FOC.Vd_injectionV;
      _motor->FOC.Vdq.q = _motor->FOC.Vdq.q - _motor->FOC.Vq_injectionV;
      Idq[1].d = _motor->FOC.Idq.d;
      Idq[1].q = _motor->FOC.Idq.q;
    }
    if (MotorState == MOTOR_STATE_RUN) {


  	  dIdq.d = (Idq[0].d - Idq[1].d);
  	  dIdq.q = (Idq[0].q - Idq[1].q);
  	  if(dIdq.q>1.0f){dIdq.q = 1.0f;}
  	  if(dIdq.q<-1.0f){dIdq.q = -1.0f;}
  	  intdidq.q = (intdidq.q + dIdq.q);
  	  if(intdidq.q>10){intdidq.q=10;}
  	  if(intdidq.q<-10){intdidq.q=-10;}

  //	  static float ffactor = 2.0f;

		_motor->FOC.IIR[0] *= (99.0f);
		_motor->FOC.IIR[1] *= (1.0f);

		_motor->FOC.IIR[0] += dIdq.d;
		_motor->FOC.IIR[1] += dIdq.q;

		_motor->FOC.IIR[0] *= 0.01f;
		_motor->FOC.IIR[1] *=0.5f;
        _motor->FOC.FOCAngle += (int)(250.0f*_motor->FOC.IIR[1] + 5.50f*intdidq.q);
    }
  }
//#endif
#ifdef USE_LR_OBSERVER
      LRObserverCollect();
#endif
#ifdef USE_ENCODER
tle5012();
#endif
//RunPLL for all angle options
_motor->FOC.angle_error = _motor->FOC.angle_error-0.02f*(int16_t)((_motor->FOC.angle_error+(int)(last_angle - _motor->FOC.FOCAngle)));
_motor->FOC.eHz = _motor->FOC.angle_error * _motor->FOC.pwm_frequency/65536.0f;
last_angle = _motor->FOC.FOCAngle;
#ifdef INTERPOLATE_V7_ANGLE
if(fabs(_motor->FOC.eHz)>0.005*_motor->FOC.pwm_frequency){
	//Only run it when there is likely to be good speed measurement stability and
	//actual utility in doing it. At low speed, there is minimal benefit, and
	//unstable speed estimation could make it worse.
_motor->FOC.FOCAngle = _motor->FOC.FOCAngle + 0.5f*_motor->FOC.angle_error;
}
#endif
 // _motor->FOC.FOCAngle = _motor->FOC.FOCAngle + _motor->FOC.angle_error;
if(MotorState==MOTOR_STATE_RUN||MotorState==MOTOR_STATE_MEASURING){
	writePWM(_motor);
	}
#ifdef LOGGING
if(lognow){
	static int post_error_samples;
	if(MotorState!=MOTOR_STATE_ERROR){
	logVars(_motor);
	post_error_samples = 50;
	}else{//If we have an error state, we want to keep the data surrounding the error log, including some sampled during and after the fault
		if(post_error_samples>1){
			logVars(_motor);
			post_error_samples--;
		}else if(post_error_samples == 1){
			print_samples_now = 1;
			post_error_samples--;
		}else{
			__NOP();
		}
	}
}
#endif
}

#define MAX_ERROR_COUNT 1

void VICheck(MESC_motor_typedef *_motor) {  // Check currents, voltages are within panic limits


  if (_motor->Raw.Iu > g_hw_setup.RawCurrLim){
	  handleError(_motor, ERROR_OVERCURRENT_PHA);
  }
  if (_motor->Raw.Iv > g_hw_setup.RawCurrLim){
	  handleError(_motor, ERROR_OVERCURRENT_PHB);
  }
  if (_motor->Raw.Iw > g_hw_setup.RawCurrLim){
	  handleError(_motor,ERROR_OVERCURRENT_PHC);
  }
  if (_motor->Raw.Vbus > g_hw_setup.RawVoltLim){
	  handleError(_motor, ERROR_OVERVOLTAGE);
  }
}
float maxIgamma;
uint16_t phasebalance;
  void ADCConversion(MESC_motor_typedef *_motor) {
	  _motor->FOC.Idq_smoothed.d = (_motor->FOC.Idq_smoothed.d*99.0f + _motor->FOC.Idq.d)*0.01f;
	  _motor->FOC.Idq_smoothed.q = (_motor->FOC.Idq_smoothed.q*99.0f + _motor->FOC.Idq.q)*0.01f;

    getRawADC(_motor);

    // Here we take the raw ADC values, offset, cast to (float) and use the
    // hardware gain values to create volt and amp variables
    //Convert the currents to real amps in SI units
	_motor->Conv.Iu =
		(float)(_motor->Raw.Iu - _motor->offset.Iu) * g_hw_setup.Igain;
	_motor->Conv.Iv =
		(float)(_motor->Raw.Iv - _motor->offset.Iv) * g_hw_setup.Igain;
	_motor->Conv.Iw =
		(float)(_motor->Raw.Iw - _motor->offset.Iw) * g_hw_setup.Igain;
	_motor->Conv.Vbus =
		(float)_motor->Raw.Vbus * g_hw_setup.VBGain;  // Vbus

    //Check for over limit conditions. We want this after the conversion so that the
    //correct overcurrent values are logged
    VICheck(_motor);

//Deal with terrible hardware choice of only having two current sensors
//Based on Iu+Iv+Iw = 0
#ifdef MISSING_UCURRSENSOR
    _motor->Conv.Iu =
    		-_motor->Conv.Iv -_motor->Conv.Iw;
#endif
#ifdef MISSING_VCURRSENSOR
    _motor->Conv.Iv =
    		-_motor->Conv.Iu -_motor->Conv.Iw;
#endif
#ifdef MISSING_WCURRSENSOR
    _motor->Conv.Iw =
    		-_motor->Conv.Iu -_motor->Conv.Iv;
#endif


    // Power Variant Clark transform
    // Here we select the phases that have the lowest duty cycle to us, since
    // they should have the best current measurements
    if (htim1.Instance->CCR1 > _motor->FOC.ADC_duty_threshold) {
      // Clark using phase V and W
      _motor->FOC.Iab[0] = -_motor->Conv.Iv -
    		  _motor->Conv.Iw;
      _motor->FOC.Iab[1] =
          one_on_sqrt3 * _motor->Conv.Iv -
          one_on_sqrt3 * _motor->Conv.Iw;
    } else if (htim1.Instance->CCR2 > _motor->FOC.ADC_duty_threshold) {
      // Clark using phase U and W
      _motor->FOC.Iab[0] = _motor->Conv.Iu;
      _motor->FOC.Iab[1] =
          -one_on_sqrt3 * _motor->Conv.Iu -
          two_on_sqrt3 * _motor->Conv.Iw;
    } else if (htim1.Instance->CCR3 > _motor->FOC.ADC_duty_threshold) {
      // Clark using phase U and V
      _motor->FOC.Iab[0] = _motor->Conv.Iu;
      _motor->FOC.Iab[1] =
          two_on_sqrt3 * _motor->Conv.Iv +
          one_on_sqrt3 * two_on_sqrt3 *
		  _motor->Conv.Iu;
    } else {
#ifdef USE_HIGHHOPES_PHASE BALANCING
		_motor->FOC.Iab[2] = _motor->Conv.Iu + _motor->Conv.Iv + _motor->Conv.Iw;
if(phasebalance){
	_motor->Conv.Iu = _motor->Conv.Iu + _motor->FOC.Iab[2];
	_motor->Conv.Iv = _motor->Conv.Iu + _motor->FOC.Iab[2];
		m_motor->Conv.Iw = _motor->Conv.Iu + _motor->FOC.Iab[2];
		}
		if(fabs(_motor->FOC.Iab[2])>fabs(maxIgamma)){
			maxIgamma = _motor->FOC.Iab[2];
		}
		if(_motor->FOC.Vdq.q<2.0f){ //Reset it to reject accumulated random noise and enable multiple goes
			maxIgamma = 0.0f;
		}
#endif
      // Do the full transform
	      _motor->FOC.Iab[0] =
	          0.66666f * _motor->Conv.Iu -
	          0.33333f * _motor->Conv.Iv -
	          0.33333f * _motor->Conv.Iw;
	      _motor->FOC.Iab[1] =
	          one_on_sqrt3 * _motor->Conv.Iv -
	          one_on_sqrt3 * _motor->Conv.Iw;
    }

    // Park
    _motor->FOC.Idq.d = _motor->FOC.sincosangle.cos * _motor->FOC.Iab[0] +
                     _motor->FOC.sincosangle.sin * _motor->FOC.Iab[1];
    _motor->FOC.Idq.q = _motor->FOC.sincosangle.cos * _motor->FOC.Iab[1] -
                     _motor->FOC.sincosangle.sin * _motor->FOC.Iab[0];
  }

  void ADCPhaseConversion(MESC_motor_typedef *_motor) {
	  //To save clock cycles in the main run loop we only want to convert the phase voltages while tracking.
  //Convert the voltages to volts in real SI units
	  _motor->Conv.Vu = (float)_motor->Raw.Vu * g_hw_setup.VBGain;
	  _motor->Conv.Vv = (float)_motor->Raw.Vv * g_hw_setup.VBGain;
	  _motor->Conv.Vw = (float)_motor->Raw.Vw * g_hw_setup.VBGain;
  }

  /////////////////////////////////////////////////////////////////////////////
  // SENSORLESS IMPLEMENTATION//////////////////////////////////////////////////
  static float Ia_last = 0.0f;
  static float Ib_last = 0.0f;
  static float La_last = 0.0f;
  static float Lb_last = 0.0f;
  static uint16_t angle = 0;

  void flux_observer(MESC_motor_typedef *_motor) {
    // LICENCE NOTE REMINDER:
    // This work deviates slightly from the BSD 3 clause licence.
    // The work here is entirely original to the MESC FOC project, and not based
    // on any appnotes, or borrowed from another project. This work is free to
    // use, as granted in BSD 3 clause, with the exception that this note must
    // be included in where this code is implemented/modified to use your
    // variable names, structures containing variables or other minor
    // rearrangements in place of the original names I have chosen, and credit
    // to David Molony as the original author must be noted.

    // With thanks to C0d3b453 for generally keeping this compiling and Elwin
    // for producing data comparing the output to a 16bit encoder.

#ifdef USE_FLUX_LINKAGE_OBSERVER
	  //Variant of the flux linkage observer created by/with Benjamin Vedder to
	  //eliminate the need to accurately know the flux linked motor parameter.
	  //This may be useful when approaching saturation; currently unclear but
	  //definitely makes setup less critical.
	  //It basically takes the normal of the flux linkage at any time and
	  //changes the flux limits accordingly, ignoring using a sqrt for computational efficiency
	  float flux_linked_norm = _motor->FOC.flux_linked_alpha*_motor->FOC.flux_linked_alpha+_motor->FOC.flux_linked_beta*_motor->FOC.flux_linked_beta;
	  float flux_err = flux_linked_norm-motor.motor_flux*motor.motor_flux;
	  motor.motor_flux = motor.motor_flux+ motor_profile->flux_linkage_gain*flux_err;
	  if(motor.motor_flux>motor_profile->flux_linkage_max){motor.motor_flux = motor_profile->flux_linkage_max;}
	  if(motor.motor_flux<motor_profile->flux_linkage_min){motor.motor_flux = motor_profile->flux_linkage_min;}
#endif
	// This is the actual observer function.
	// We are going to integrate Va-Ri and clamp it positively and negatively
	// the angle is then the arctangent of the integrals shifted 180 degrees
#ifdef USE_SALIENT_OBSERVER
	  float La, Lb;
	  getLabFast(_motor->FOC.FOCAngle, motor.Lphase, motor.Lqd_diff, &La, &Lb);

	  _motor->FOC.flux_linked_alpha = _motor->FOC.flux_linked_alpha +
			  (_motor->FOC.Vab[0] - motor.Rphase * _motor->FOC.Iab[0])*_motor->FOC.pwm_period -
        La * (_motor->FOC.Iab[0] - Ia_last) - //Salient inductance NOW
		_motor->FOC.Iab[0] * (La - La_last); //Differential of phi = Li -> Ldi/dt+idL/dt
	  _motor->FOC.flux_linked_beta = _motor->FOC.flux_linked_beta +
			  (_motor->FOC.Vab[1] - motor.Rphase * _motor->FOC.Iab[1])*_motor->FOC.pwm_period -
        Lb * (_motor->FOC.Iab[1] - Ib_last) -
		_motor->FOC.Iab[1] * (Lb-Lb_last);
//Store the inductances
    La_last = La;
    Lb_last = Lb;
#else
	  _motor->FOC.flux_linked_alpha =
			  _motor->FOC.flux_linked_alpha + (_motor->FOC.Vab[0] - motor.Rphase * _motor->FOC.Iab[0])*_motor->FOC.pwm_period-
        motor.Lphase * (_motor->FOC.Iab[0] - Ia_last);
	  _motor->FOC.flux_linked_beta =
			  _motor->FOC.flux_linked_beta + (_motor->FOC.Vab[1] - motor.Rphase * _motor->FOC.Iab[1])*_motor->FOC.pwm_period -
        motor.Lphase * (_motor->FOC.Iab[1] - Ib_last);
#endif
//Store the currents
    Ia_last = _motor->FOC.Iab[0];
    Ib_last = _motor->FOC.Iab[1];

#ifdef USE_NONLINEAR_OBSERVER_CENTERING
///Try directly applying the centering using the same method as the flux linkage observer
    float err = motor.motor_flux*motor.motor_flux-_motor->FOC.flux_linked_alpha*_motor->FOC.flux_linked_alpha-_motor->FOC.flux_linked_beta*_motor->FOC.flux_linked_beta;
    _motor->FOC.flux_linked_beta = _motor->FOC.flux_linked_beta+err*_motor->FOC.flux_linked_beta*motor_profile->non_linear_centering_gain;
    _motor->FOC.flux_linked_alpha = _motor->FOC.flux_linked_alpha+err*_motor->FOC.flux_linked_alpha*motor_profile->non_linear_centering_gain;
#endif
#ifdef USE_CLAMPED_OBSERVER_CENTERING
    if (_motor->FOC.flux_linked_alpha > motor.motor_flux) {
    	_motor->FOC.flux_linked_alpha = motor.motor_flux;}
    if (_motor->FOC.flux_linked_alpha < -motor.motor_flux) {
    	_motor->FOC.flux_linked_alpha = -motor.motor_flux;}
    if (_motor->FOC.flux_linked_beta > motor.motor_flux) {
    	_motor->FOC.flux_linked_beta = motor.motor_flux;}
    if (_motor->FOC.flux_linked_beta < -motor.motor_flux) {
    	_motor->FOC.flux_linked_beta = -motor.motor_flux;}
#endif

#ifdef USE_ENCODER
    //This does not apply the encoder angle,
    //It tracks the difference between the encoder and the observer.
    _motor->FOC.enc_obs_angle = angle - _motor->FOC.enc_angle;
#else
    if(_motor->FOC.inject==0){
    _motor->FOC.FOCAngle = (uint16_t)(32768.0f + 10430.0f * fast_atan2(_motor->FOC.flux_linked_beta, _motor->FOC.flux_linked_alpha)) - 32768;
    }
#endif
  }

  // fast_atan2 based on https://math.stackexchange.com/a/1105038/81278
  // Via Odrive project
  // https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/utils.cpp
  // This function is MIT licenced, copyright Oskar Weigl/Odrive Robotics
  // The origin for Odrive atan2 is public domain. Thanks to Odrive for making
  // it easy to borrow.
  float min(float lhs, float rhs) { return (lhs < rhs) ? lhs : rhs; }
  float max(float lhs, float rhs) { return (lhs > rhs) ? lhs : rhs; }

  float fast_atan2(float y, float x) {
    // a := min (|x|, |y|) / max (|x|, |y|)
    float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    // inject FLT_MIN in denominator to avoid division by zero
    float a = min(abs_x, abs_y) / (max(abs_x, abs_y));
    // s := a * a
    float s = a * a;
    // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
    float r =
        ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    // if |y| > |x| then r := 1.57079637 - r
    if (abs_y > abs_x) r = 1.57079637f - r;
    // if x < 0 then r := 3.14159274 - r
    if (x < 0.0f) r = 3.14159274f - r;
    // if y < 0 then r := -r
    if (y < 0.0f) r = -r;

    return r;
  }

  /////////////////////////////////////////////////////////////////////////////
  ////////Hall Sensor Implementation///////////////////////////////////////////
  static float dir = 1;
  static uint16_t current_hall_angle;
  static int last_hall_state;
  static uint16_t last_hall_angle;
  static float ticks_since_last_observer_change = 65535;
  static float last_observer_period = 65536;
  static float one_on_last_observer_period = 1;
  float angular_velocity = 0;
  static float angle_step = 0;

  static int hall_error = 0;
  void hallAngleEstimator(MESC_motor_typedef *_motor) {  // Implementation using the mid point of the hall
                               // sensor angles, which should be much more
                               // reliable to generate that the edges

    if (current_hall_state != last_hall_state) {
      _motor->FOC.hall_update = 1;
      if (current_hall_state == 0) {
        MotorState = MOTOR_STATE_ERROR;
        MotorError = MOTOR_ERROR_HALL0;
      } else if (current_hall_state == 7) {
        MotorState = MOTOR_STATE_ERROR;
        MotorError = MOTOR_ERROR_HALL7;
      }
      //////////Implement the Hall table here, but the vector can be dynamically
      /// created/filled by another function/////////////
      current_hall_angle = _motor->FOC.hall_table[current_hall_state - 1][2];

      // Calculate Hall error

      uint16_t a;
      if ((a = current_hall_angle - last_hall_angle) < 32000)  // Forwards
      {
        hall_error =
            _motor->FOC.FOCAngle - _motor->FOC.hall_table[current_hall_state - 1][0];
        dir = 1.0f;
        // _motor->FOC.HallAngle = _motor->FOC.HallAngle - 5460;
      } else  // Backwards
      {
        hall_error =
            _motor->FOC.FOCAngle - _motor->FOC.hall_table[current_hall_state - 1][1];
        dir = -1.0f;
        // _motor->FOC.HallAngle = _motor->FOC.HallAngle + 5460;
      }
      if (hall_error > 32000) {
        hall_error = hall_error - 65536;
      }
      if (hall_error < -32000) {
        hall_error = hall_error + 65536;
      }
    }
  }

  void angleObserver(MESC_motor_typedef *_motor) {
    // This function should take the available data (hall change, BEMF crossing
    // etc...) and process it with a PLL type mechanism
    if (_motor->FOC.hall_update == 1) {
      _motor->FOC.hall_update = 0;
      last_observer_period = ticks_since_last_observer_change;
      float one_on_ticks = (1.0f / ticks_since_last_observer_change);
      one_on_last_observer_period =
          (4.0f * one_on_last_observer_period + (one_on_ticks)) * 0.2f;  // ;
      angle_step =
          (4.0f * angle_step +
           (one_on_ticks)*_motor->FOC.hall_table[last_hall_state - 1][3]) *
          0.2f;

      // Reset the counters, track the previous state
      last_hall_state = current_hall_state;
      last_hall_angle = current_hall_angle;
      ticks_since_last_observer_change = 0;
    }

    // Run the counter
    ticks_since_last_observer_change = ticks_since_last_observer_change + 1;

    if (ticks_since_last_observer_change <= 2.0f * last_observer_period) {
      /*      _motor->FOC.FOCAngle = _motor->FOC.FOCAngle + (uint16_t)(dir*angle_step
         + one_on_last_observer_period * (-0.9f * hall_error)); //Does not
         work...
           //Why?
 */
      if (dir > 0.0f) {  // Apply a gain to the error as well as the feed forward
        // from the last hall period. Gain of 0.9-1.1 seems to work
        // well when using corrected hall positions and spacings
        _motor->FOC.FOCAngle =
            _motor->FOC.FOCAngle +
            (uint16_t)(angle_step - one_on_last_observer_period * hall_error);
        // one_on_last_observer_period * (-0.2f * hall_error));
      } else if (dir < 0.0f) {
        _motor->FOC.FOCAngle =
            _motor->FOC.FOCAngle +
            (uint16_t)(-angle_step +
                       one_on_last_observer_period * (-0.9f * hall_error));
        // Also does not work,
        // Why??
        _motor->FOC.FOCAngle =
            _motor->FOC.FOCAngle -
            (uint16_t)(angle_step +
                       one_on_last_observer_period * (0.2f * hall_error));
      }
    }
    if (ticks_since_last_observer_change > 1500.0f) {
      ticks_since_last_observer_change = 1500.0f;
      last_observer_period = 1500.0f;  //(ticks_since_last_hall_change);
      one_on_last_observer_period =
          1.0f / last_observer_period;  // / ticks_since_last_hall_change;
      _motor->FOC.FOCAngle = current_hall_angle;
    }
  }

  void OLGenerateAngle(MESC_motor_typedef *_motor) {

    _motor->FOC.FOCAngle = _motor->FOC.FOCAngle + _motor->FOC.openloop_step;
    // ToDo
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // FOC PID algorithms
  //////////////////////////////////////////////////////////////////////////////////////////
  float FW_current = 0;

  void MESCFOC(MESC_motor_typedef *_motor) {

    // Here we are going to do a PID loop to control the dq currents, converting
    // Idq into Vdq Calculate the errors
    static MESCiq_s Idq_err;
#ifdef USE_FIELD_WEAKENINGV2
    if((FW_current<_motor->FOC.Idq_req.d)&&(MotorState==MOTOR_STATE_RUN)){//Field weakenning is -ve, but there may already be d-axis from the MTPA
    	Idq_err.d = (FW_current - _motor->FOC.Idq.d) * _motor->FOC.Id_pgain;
    }else{
    	Idq_err.d = (_motor->FOC.Idq_req.d - _motor->FOC.Idq.d) * _motor->FOC.Id_pgain;
    }
#else
	Idq_err.d = (_motor->FOC.Idq_req.d - _motor->FOC.Idq.d) * _motor->FOC.Id_pgain;
#endif
    Idq_err.q = (_motor->FOC.Idq_req.q - _motor->FOC.Idq.q) * _motor->FOC.Iq_pgain;

    // Integral error
    _motor->FOC.Idq_int_err.d =
    		_motor->FOC.Idq_int_err.d + _motor->FOC.Id_igain * Idq_err.d * _motor->FOC.pwm_period;
    _motor->FOC.Idq_int_err.q =
    		_motor->FOC.Idq_int_err.q + _motor->FOC.Iq_igain * Idq_err.q * _motor->FOC.pwm_period;
    // Apply the integral gain at this stage to enable bounding it


        // Apply the PID, and potentially smooth the output for noise - sudden
      // changes in VDVQ may be undesirable for some motors. Integral error is
      // pre-bounded to avoid integral windup, proportional gain needs to have
      // effect even at max integral to stabilise and avoid trips
      _motor->FOC.Vdq.d = Idq_err.d + _motor->FOC.Idq_int_err.d;
      _motor->FOC.Vdq.q = Idq_err.q + _motor->FOC.Idq_int_err.q;

      // Bounding final output

#if defined(USE_SQRT_CIRCLE_LIM)
      float Vmagnow2 = _motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q;
      //Check if the vector length is greater than the available voltage
      if(Vmagnow2>_motor->FOC.Vmag_max2){
		  float Vmagnow = sqrtf(Vmagnow2);
		  float one_on_Vmagnow = 1/Vmagnow;
		  float one_on_VmagnowxVmagmax = _motor->FOC.Vmag_max*one_on_Vmagnow;
		  _motor->FOC.Vdq.d = _motor->FOC.Vdq.d*one_on_VmagnowxVmagmax;
		  _motor->FOC.Vdq.q = _motor->FOC.Vdq.q*one_on_VmagnowxVmagmax;
		  _motor->FOC.Idq_int_err.d = _motor->FOC.Idq_int_err.d*one_on_VmagnowxVmagmax;
		  _motor->FOC.Idq_int_err.q = _motor->FOC.Idq_int_err.q*one_on_VmagnowxVmagmax;
      }
#elif defined(USE_SQRT_CIRCLE_LIM_VD)
     //Circle limiter that favours Vd, similar to used in VESC, and as an option in ST firmware.for torque
      //This method was primarily designed for induction motors, where the d axis is required to
      //make the magnetic field for torque. Nevertheless, this finds application at extreme currents and
      //during field weakening.
      //Latent concerns about the usual implementation that allows ALL the voltage to be
      //assigned to Vd becoming unstable as the angle relative to the rotor exceeds 45 degrees
      //due to rapidly collapsing q-axis voltage. Therefore, this option will be allowed, but
      // with a limit of voltage angle 60degrees (sin60 = 0.866) from the rotor.

      if(_motor->FOC.Vdq.d<-0.866f*_motor->FOC.Vmag_max){ //Negative values of Vd - Normally Vd is -ve since it is driving field advance
		  _motor->FOC.Vdq.d = -0.866f*_motor->FOC.Vmag_max; //Hard clamp the Vd
		  if(_motor->FOC.Idq_int_err.d<_motor->FOC.Vdq.d){
			  _motor->FOC.Idq_int_err.d = _motor->FOC.Vdq.d; //Also clamp the integral to stop windup
		  }
      } else if(_motor->FOC.Vdq.d>0.866f*_motor->FOC.Vmag_max){ //Positive values of Vd
    	  _motor->FOC.Vdq.d = 0.866f*_motor->FOC.Vmag_max; //Hard clamp the Vd
		  if(_motor->FOC.Idq_int_err.d>_motor->FOC.Vdq.d){
			  _motor->FOC.Idq_int_err.d = _motor->FOC.Vdq.d; //Also clamp the integral to stop windup
		  }
      }

      //Now we take care of the overall length of the voltage vector
      float Vmagnow2 = _motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q;
      if(Vmagnow2>_motor->FOC.Vmag_max2){
    	  if(_motor->FOC.Vdq.q>0){ //Positive Vq
    		  _motor->FOC.Vdq.q = sqrtf(_motor->FOC.Vmag_max2-_motor->FOC.Vdq.d*_motor->FOC.Vdq.d);
    		  if(_motor->FOC.Idq_int_err.q>_motor->FOC.Vdq.q){
    			  _motor->FOC.Idq_int_err.q = _motor->FOC.Vdq.q;
    		  }
    	  }
    	  else{ //Negative Vq
    		  _motor->FOC.Vdq.q = -sqrtf(_motor->FOC.Vmag_max2-_motor->FOC.Vdq.d*_motor->FOC.Vdq.d);
			  if(_motor->FOC.Idq_int_err.q<_motor->FOC.Vdq.q){
				  _motor->FOC.Idq_int_err.q = _motor->FOC.Vdq.q;
			  }
    	  }
      }

#else
      // These limits are experimental, but result in close to 100% modulation.
      // Since Vd and Vq are orthogonal, limiting Vd is not especially helpful
      // in reducing overall voltage magnitude, since the relation
      // Vout=(Vd^2+Vq^2)^0.5 results in Vd having a small effect. Vd is
      // primarily used to drive the resistive part of the field; there is no
      // BEMF pushing against Vd and so it does not scale with RPM (except for
      // cross coupling).

      // Bounding integral
        if (_motor->FOC.Idq_int_err.d > _motor->FOC.Vdint_max){_motor->FOC.Idq_int_err.d = _motor->FOC.Vdint_max;}
        if (_motor->FOC.Idq_int_err.d < -_motor->FOC.Vdint_max){_motor->FOC.Idq_int_err.d = -_motor->FOC.Vdint_max;}
        if (_motor->FOC.Idq_int_err.q > _motor->FOC.Vqint_max){_motor->FOC.Idq_int_err.q = _motor->FOC.Vqint_max;}
        if (_motor->FOC.Idq_int_err.q < -_motor->FOC.Vqint_max){_motor->FOC.Idq_int_err.q = -_motor->FOC.Vqint_max;}
      //Bounding output
      if (_motor->FOC.Vdq.d > _motor->FOC.Vd_max)
        (_motor->FOC.Vdq.d = _motor->FOC.Vd_max);
      if (_motor->FOC.Vdq.d < -_motor->FOC.Vd_max)
        (_motor->FOC.Vdq.d = -_motor->FOC.Vd_max);
      if (_motor->FOC.Vdq.q > _motor->FOC.Vq_max)
        (_motor->FOC.Vdq.q = _motor->FOC.Vq_max);
      if (_motor->FOC.Vdq.q < -_motor->FOC.Vq_max)
        (_motor->FOC.Vdq.q = -_motor->FOC.Vq_max);
#endif
#ifdef USE_FIELD_WEAKENINGV2
      //Calculate the module of voltage applied,
      Vmagnow2 = _motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q; //Need to recalculate this since limitation has maybe been applied
      //Apply a linear slope from the threshold to the max module
      //Step towards with exponential smoother
      if(Vmagnow2>(_motor->FOC.field_weakening_threshold*_motor->FOC.field_weakening_threshold)){
    	  FW_current = 0.95f*FW_current +
    			  	  	0.05f*_motor->FOC.field_weakening_curr_max *_motor->FOC.field_weakening_multiplier*
						(_motor->FOC.field_weakening_threshold - sqrtf(Vmagnow2));
      }else{
    	  FW_current*=0.95f;//Ramp down a bit slowly
		  if(FW_current>0.1f){//We do not allow positive field weakening current, and we want it to actually go to zero eventually
			  FW_current = 0.0f;
		  }
      }
      //Apply the field weakening only if the additional d current is greater than the requested d current

#endif
    }


  static float mid_value = 0;
  float top_value;
  float bottom_value;
  uint16_t t_phase, b_phase;
 uint16_t deadtime_comp = DEADTIME_COMP_V;

 void writePWM(MESC_motor_typedef *_motor) {

    // Now we update the sin and cos values, since when we do the inverse
    // transforms, we would like to use the most up to date versions(or even the
    // next predicted version...)
	sin_cos_fast(_motor->FOC.FOCAngle, &_motor->FOC.sincosangle.sin, &_motor->FOC.sincosangle.cos);

    // Inverse Park transform
    _motor->FOC.Vab[0] = _motor->FOC.sincosangle.cos * _motor->FOC.Vdq.d -
                      _motor->FOC.sincosangle.sin * _motor->FOC.Vdq.q;
    _motor->FOC.Vab[1] = _motor->FOC.sincosangle.sin * _motor->FOC.Vdq.d +
                      _motor->FOC.sincosangle.cos * _motor->FOC.Vdq.q;
    _motor->FOC.Vab[2] = 0.0f;

	// Inverse Clark transform - power variant
	_motor->FOC.inverterVoltage[0] = _motor->FOC.Vab[0];
	_motor->FOC.inverterVoltage[1] = -0.5f*_motor->FOC.inverterVoltage[0];
	_motor->FOC.inverterVoltage[2] = _motor->FOC.inverterVoltage[1] - sqrt3_on_2 * _motor->FOC.Vab[1];
	_motor->FOC.inverterVoltage[1] = _motor->FOC.inverterVoltage[1] + sqrt3_on_2 * _motor->FOC.Vab[1];

    ////////////////////////////////////////////////////////
    // SVPM implementation
    // Try to do this as a "midpoint clamp" where rather than finding the
    // lowest, we find the highest and lowest and subtract the middle
    top_value = _motor->FOC.inverterVoltage[0];
    bottom_value = top_value;
    b_phase = 0;
    t_phase = 0;

    if (_motor->FOC.inverterVoltage[1] > top_value) {
      top_value = _motor->FOC.inverterVoltage[1];
      t_phase = 1;
    }
    if (_motor->FOC.inverterVoltage[2] > top_value) {
      top_value = _motor->FOC.inverterVoltage[2];
      t_phase = 2;
    }
    if (_motor->FOC.inverterVoltage[1] < bottom_value) {
      bottom_value = _motor->FOC.inverterVoltage[1];
      b_phase = 1;
    }
    if (_motor->FOC.inverterVoltage[2] < bottom_value) {
      bottom_value = _motor->FOC.inverterVoltage[2];
      b_phase = 2;
    }
#ifdef SEVEN_SECTOR
    mid_value = _motor->FOC.PWMmid -
                0.5f * _motor->FOC.Vab_to_PWM * (top_value + bottom_value);

    ////////////////////////////////////////////////////////
    // Actually write the value to the timer registers
    _motor->mtimer->Instance->CCR1 =
    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[0] + mid_value);
    _motor->mtimer->Instance->CCR2 =
    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[1] + mid_value);
    _motor->mtimer->Instance->CCR3 =
    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[2] + mid_value);

    //Dead time compensation
#ifdef DEADTIME_COMP
    // LICENCE NOTE:
    	  // This function deviates slightly from the BSD 3 clause licence.
    	  // The work here is entirely original to the MESC FOC project, and not based
    	  // on any appnotes, or borrowed from another project. This work is free to
    	  // use, as granted in BSD 3 clause, with the exception that this note must
    	  // be included in where this code is implemented/modified to use your
    	  // variable names, structures containing variables or other minor
    	  // rearrangements in place of the original names I have chosen, and credit
    	  // to David Molony as the original author must be noted.
    //The problem with dead time, is that it is essentially a voltage tie through the body diodes to VBus or ground, depending on the current direction.
    //If we know the direction of current, and the effective dead time length we can remove this error, by writing the corrected voltage.
    //This is observed to improve sinusoidalness of currents, but has a slight audible buzz
    //When the current is approximately zero, it is hard to resolve the direction, and therefore the compensation is ineffective.
    //However, no torque is generated when the current and voltage are close to zero, so no adverse performance except the buzz.
    if(_motor->Conv.Iu < -0.030f){_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->CCR1-deadtime_comp;}
    if(_motor->Conv.Iv < -0.030f){_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->CCR2-deadtime_comp;}
    if(_motor->Conv.Iw < -0.030f){_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->CCR3-deadtime_comp;}
    if(_motor->Conv.Iu > -0.030f){_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->CCR1+deadtime_comp;}
    if(_motor->Conv.Iv > -0.030f){_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->CCR2+deadtime_comp;}
    if(_motor->Conv.Iw > -0.030f){_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->CCR3+deadtime_comp;}

#endif
#else //Use 5 sector, bottom clamp implementation
//ToDo, threshold for turning on sinusoidal modulation
    _motor->FOC.inverterVoltage[0] = _motor->FOC.inverterVoltage[0]-bottom_value;
    _motor->FOC.inverterVoltage[1] = _motor->FOC.inverterVoltage[1]-bottom_value;
    _motor->FOC.inverterVoltage[2] = _motor->FOC.inverterVoltage[2]-bottom_value;

    _motor->mtimer->Instance->CCR1 = (uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[0]);
    _motor->mtimer->Instance->CCR2 = (uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[1]);
    _motor->mtimer->Instance->CCR3 = (uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[2]);
#ifdef OVERMOD_DT_COMP_THRESHOLD
    //Concept here is that if we are close to the VBus max, we just do not turn the FET off.
    //Set CCRx to ARR, record how much was added, then next cycle, remove it from the count.
    //If the duty is still above the threshold, the CCR will still be set to ARR, until the duty request is sufficiently low...
static int carryU, carryV, carryW;

	_motor->mtimer->Instance->CCR1 = 	_motor->Instance->CCR1 - carryU;
	_motor->mtimer->Instance->CCR2 = 	_motor->mtimer->Instance->CCR2 - carryV;
	_motor->mtimer->Instance->CCR3 = 	_motor->mtimer->Instance->CCR3 - carryW;
	carryU = 0;
	carryV = 0;
	carryW = 0;

	if(_motor->mtimer->Instance->CCR1>(_motor->mtimer->Instance->ARR-OVERMOD_DT_COMP_THRESHOLD)){
		carryU = _motor->mtimer->Instance->ARR-_motor->mtimer->Instance->CCR1; //Save the amount we have overmodulated by
		_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->ARR;
	}
	if(_motor->mtimer->Instance->CCR2>(_motor->mtimer->Instance->ARR-OVERMOD_DT_COMP_THRESHOLD)){
		carryV = _motor->mtimer->Instance->ARR-_motor->mtimer->Instance->CCR2; //Save the amount we have overmodulated by
		_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->ARR;
	}
	if(_motor->mtimer->Instance->CCR3>(_motor->mtimer->Instance->ARR-OVERMOD_DT_COMP_THRESHOLD)){
		carryW = _motor->mtimer->Instance->ARR-_motor->mtimer->Instance->CCR3; //Save the amount we have overmodulated by
		_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->ARR;
	}
#endif
#endif


  }

  // Here we set all the PWMoutputs to LOW, without triggering the timerBRK,
  // which should only be set by the hardware comparators, in the case of a
  // shoot-through or other catastrophic event This function means that the
  // timer can be left running, ADCs sampling etc which enables a recovery, or
  // single PWM period break in which the backEMF can be measured directly
  // This function needs implementing and testing before any high current or
  // voltage is applied, otherwise... DeadFETs
  void generateBreak(MESC_motor_typedef *_motor) {
    phU_Break(_motor);
    phV_Break(_motor );
    phW_Break(_motor );
  }
  void generateEnable(MESC_motor_typedef *_motor) {
    phU_Enable(_motor);
    phV_Enable(_motor);
    phW_Enable(_motor);
  }

  static float top_V;
  static float bottom_V;
  static float top_I;
  static float bottom_I;
  static float count_top;
  static float count_topq;
  static float count_bottom;
  static float count_bottomq;

  static float Vd_temp;
  static float Vq_temp;
  static float top_I_L;
  static float bottom_I_L;
  static float top_I_Lq;
  static float bottom_I_Lq;
  static int PWM_cycles = 0;

  void measureResistance(MESC_motor_typedef *_motor) {

    if (PWM_cycles < 2) {
      uint16_t half_ARR = htim1.Instance->ARR / 2;
      htim1.Instance->CCR1 = half_ARR;
      htim1.Instance->CCR2 = half_ARR;
      htim1.Instance->CCR3 = half_ARR;
      motor.Rphase = 0.001f;     // Initialise with a very low value 1mR
      motor.Lphase = 0.000001f;  // Initialise with a very low value 1uH
      motor.Lqphase = 0.000001f;
      calculateVoltageGain(_motor);    // Set initial gains to enable MESCFOC to run
      calculateGains(_motor);
      phU_Enable(_motor);
      phV_Enable(_motor);
      phW_Enable(_motor);
      _motor->FOC.Idq_req.d = I_MEASURE;
      _motor->FOC.Idq_req.q = 0.0f;
      _motor->FOC.FOCAngle = 0;

      _motor->FOC.inject = 0;  // flag to not inject at SVPWM top

      MESCFOC(_motor);
      writePWM(_motor);

      top_V = 0;
      bottom_V = 0;
      top_I = 0;
      bottom_I = 0;
      top_I_L = 0;
      bottom_I_L = 0;
      top_I_Lq = 0;
      bottom_I_Lq = 0;

      count_top = 0.0f;
      count_bottom = 0.0f;
    }

    else if (PWM_cycles < 35000) {  // Align the rotor for ~1 second
      _motor->FOC.Idq_req.d = I_MEASURE;
      _motor->FOC.Idq_req.q = 0.0f;

      _motor->FOC.inject = 0;
      MESCFOC(_motor);
      writePWM(_motor);
    }

    else if (PWM_cycles < 40000) {  // Lower setpoint
      _motor->FOC.Idq_req.d = 0.20f*I_MEASURE;
      _motor->FOC.inject = 0;
      MESCFOC(_motor);
      writePWM(_motor);

      bottom_V = bottom_V + _motor->FOC.Vdq.d;
      bottom_I = bottom_I + _motor->FOC.Idq.d;
      count_bottom++;
      Vd_temp = _motor->FOC.Vdq.d * 1.0f;  // Store the voltage required for the low setpoint, to
                       	   	   	   	   	 // use as an offset for the inductance
    }

    else if (PWM_cycles < 45000) {  // Upper setpoint stabilisation
      _motor->FOC.Idq_req.d = I_MEASURE;
      _motor->FOC.inject = 0;
      MESCFOC(_motor);
      writePWM(_motor);

    }

    else if (PWM_cycles < 50000) {  // Upper setpoint
      _motor->FOC.Idq_req.d = I_MEASURE;
      _motor->FOC.inject = 0;
      MESCFOC(_motor);
      writePWM(_motor);

      top_V = top_V + _motor->FOC.Vdq.d;
      top_I = top_I + _motor->FOC.Idq.d;
      count_top++;
    } else if (PWM_cycles < 50001) {  // Calculate R

      generateBreak(_motor);
      motor.Rphase = (top_V - bottom_V) / (top_I - bottom_I);

      //Initialise the variables for the next measurement
      //Vd_temp = _motor->FOC.Vdq.d * 1.0f;  // Store the voltage required for the high setpoint, to
                       	   	   	   	   	 // use as an offset for the inductance
      Vq_temp = 0.0f;
      _motor->FOC.Vdq.q = 0.0f;//
      _motor->FOC.Idq_int_err.d = 0.0f;
      _motor->FOC.Idq_int_err.q = 0.0f;
      count_top = 0.0f;
      count_bottom = 0.0f;
      top_I_L = 0.0f;
      bottom_I_L = 0.0f;

      generateEnable(_motor);
    }
/////////////////////////// Collect Ld variable//////////////////////////
    else if (PWM_cycles < 80001) {
      // generateBreak();
      _motor->FOC.inject = 1;  // flag to the SVPWM writer to inject at top
      _motor->FOC.Vd_injectionV = V_MEASURE;
      _motor->FOC.Vq_injectionV = 0.0f;

      _motor->FOC.Vdq.d = Vd_temp;
      _motor->FOC.Vdq.q = 0.0f;


      if (_motor->FOC.inject_high_low_now == 1) {
        top_I_L = top_I_L + _motor->FOC.Idq.d;
        count_top++;
      } else if (_motor->FOC.inject_high_low_now == 0) {
        bottom_I_L = bottom_I_L + _motor->FOC.Idq.d;
        count_bottom++;
      }
    }

    else if (PWM_cycles < 80002) {
      generateBreak(_motor);
      motor.Lphase =
          fabsf((_motor->FOC.Vd_injectionV) /
          ((top_I_L - bottom_I_L) / (count_top * _motor->FOC.pwm_period)));
      top_I_Lq = 0.0f;
      bottom_I_Lq = 0.0f;
      count_topq = 0.0f;
      count_bottomq = 0.0f;
      __NOP();  // Put a break point on it...
    } else if (PWM_cycles < 80003) {
      phU_Enable(_motor);
      phV_Enable(_motor);
      phW_Enable(_motor);

////////////////////////// Collect Lq variable//////////////////////////////
    } else if (PWM_cycles < 100003) {
      //			generateBreak();
      _motor->FOC.Vd_injectionV = 0.0f;
      _motor->FOC.Vq_injectionV = V_MEASURE;
      _motor->FOC.inject = 1;  // flag to the SVPWM writer to update at top
      _motor->FOC.Vdq.d = Vd_temp;  // Vd_temp to keep it aligned with D axis
      _motor->FOC.Vdq.q = 0.0f;


      if (_motor->FOC.inject_high_low_now == 1) {
        top_I_Lq = top_I_Lq + _motor->FOC.Idq.q;
        count_topq++;
      } else if (_motor->FOC.inject_high_low_now == 0) {
        bottom_I_Lq = bottom_I_Lq + _motor->FOC.Idq.q;
        count_bottomq++;
      }
    }

    else {
      generateBreak(_motor);
      motor.Lqphase =
          fabsf((_motor->FOC.Vq_injectionV) /
          ((top_I_Lq - bottom_I_Lq) / (count_top * _motor->FOC.pwm_period)));

      MotorState = MOTOR_STATE_IDLE;
      motor.uncertainty = 0;

      _motor->FOC.inject = 0;  // flag to the SVPWM writer stop injecting at top
      _motor->FOC.Vd_injectionV = HFI_VOLTAGE;
      _motor->FOC.Vq_injectionV = 0.0f;
      calculateGains(_motor);
      MotorState = MOTOR_STATE_TRACKING;
      PWM_cycles = 0;
      phU_Enable(_motor);
      phV_Enable(_motor);
      phW_Enable(_motor);
    }
    PWM_cycles++;
  }


  void getHallTable(MESC_motor_typedef *_motor) {
    static int firstturn = 1;
    static int hallstate;
    hallstate = getHallState();
    static int lasthallstate = -1;
    static uint16_t pwm_count = 0;
    static int anglestep = 5;  // This defines how fast the motor spins
    static uint32_t hallangles[7][2];
    static int rollover;
    hallstate = current_hall_state;
    if (firstturn) {
    	generateEnable(_motor);
      lasthallstate = hallstate;
      (void)lasthallstate;
      firstturn = 0;
    }

    ////// Align the rotor////////////////////
    static uint16_t a = 65535;
    if (a)  // Align time
    {
      _motor->FOC.Idq_req.d = 10.0f;
      _motor->FOC.Idq_req.q = 0.0f;

      _motor->FOC.FOCAngle = 0.0f;
      a = a - 1;
    } else {
      _motor->FOC.Idq_req.d = 10.0f;
      _motor->FOC.Idq_req.q = 0.0f;
      static int dir = 1;
      if (pwm_count < 65534) {
        if (_motor->FOC.FOCAngle < (anglestep)) {
          rollover = hallstate;
        }
        if ((_motor->FOC.FOCAngle < (30000)) &&
            (_motor->FOC.FOCAngle > (29000 - anglestep))) {
          rollover = 0;
        }
        lasthallstate = hallstate;
        if (rollover == hallstate) {
          hallangles[hallstate][0] =
              hallangles[hallstate][0] +
              (uint32_t)65535;  // Accumulate the angles through the sweep
        }

        _motor->FOC.FOCAngle =
            _motor->FOC.FOCAngle + anglestep;  // Increment the angle
        hallangles[hallstate][0] =
            hallangles[hallstate][0] +
            _motor->FOC.FOCAngle;       // Accumulate the angles through the sweep
        hallangles[hallstate][1]++;  // Accumulate the number of PWM pulses for
                                     // this hall state
        pwm_count = pwm_count + 1;
      } else if (pwm_count < 65535) {
        if (dir == 1) {
          dir = 0;
          rollover = 0;
        }
        if ((_motor->FOC.FOCAngle < (12000)) && (hallstate != last_hall_state)) {
          rollover = hallstate;
        }
        if ((_motor->FOC.FOCAngle < (65535)) &&
            (_motor->FOC.FOCAngle > (65535 - anglestep))) {
          rollover = 0;
        }
        lasthallstate = hallstate;
        if (rollover == hallstate) {
          hallangles[hallstate][0] =
              hallangles[hallstate][0] +
              (uint32_t)65535;  // Accumulate the angles through the sweep
        }

        _motor->FOC.FOCAngle =
            _motor->FOC.FOCAngle - anglestep;  // Increment the angle
        hallangles[hallstate][0] =
            hallangles[hallstate][0] +
            _motor->FOC.FOCAngle;       // Accumulate the angles through the sweep
        hallangles[hallstate][1]++;  // Accumulate the number of PWM pulses for
                                     // this hall state
        pwm_count = pwm_count + 1;
      }
    }
    if (pwm_count == 65535) {
      generateBreak(_motor);  // Debugging
      for (int i = 1; i < 7; i++) {
        hallangles[i][0] = hallangles[i][0] / hallangles[i][1];
        if (hallangles[i][0] > 65535) {
          hallangles[i][0] = hallangles[i][0] - 65535;
        }
      }
      for (int i = 0; i < 6; i++) {
            _motor->FOC.hall_table[i][2] = hallangles[i + 1][0];//This is the center angle of the hall state
            _motor->FOC.hall_table[i][3] = hallangles[i + 1][1];//This is the width of the hall state
            _motor->FOC.hall_table[i][0] = _motor->FOC.hall_table[i][2]-_motor->FOC.hall_table[i][3]/2;//This is the start angle of the hall state
            _motor->FOC.hall_table[i][1] = _motor->FOC.hall_table[i][2]+_motor->FOC.hall_table[i][3]/2;//This is the end angle of the hall state
      }
      MotorState = MOTOR_STATE_RUN;
      _motor->FOC.Idq_req.d = 0;
      _motor->FOC.Idq_req.q = 0;
      phU_Enable(_motor);
      phV_Enable(_motor);
      phW_Enable(_motor);
    }
  }

  void measureInductance(MESC_motor_typedef *_motor){  // UNUSED, THIS HAS BEEN ROLLED INTO THE MEASURE
                            // RESISTANCE... no point in 2 functions really...
__NOP();
  }
  int angle_delta;
  static volatile float temp_flux;
  static volatile float temp_FLA;
  static volatile float temp_FLB;

  void getkV(MESC_motor_typedef *_motor) {
  	_motor->FOC.inject = 0;
    static int cycles = 0;

    if (cycles < 2) {
    	motor_profile->flux_linkage_max = 0.1f;
    	motor_profile->flux_linkage_min = 0.00001f;//Set really wide limits
    	_motor->FOC.openloop_step = 0;
    	motor.motor_flux = motor_profile->flux_linkage_max;
        phU_Enable(_motor);
        phV_Enable(_motor);
        phW_Enable(_motor);
    }

    flux_observer(_motor);//We run the flux observer during this

    static int count = 0;
    static uint16_t temp_angle;
    if (cycles < 60002) {
        _motor->FOC.Idq_req.d = I_MEASURE*0.5f;  //
        _motor->FOC.Idq_req.q = 0.0f;
    	angle_delta = temp_angle-_motor->FOC.FOCAngle;
    	_motor->FOC.openloop_step = (uint16_t)(ERPM_MEASURE*65536.0f/(_motor->FOC.pwm_frequency*60.0f)*(float)cycles/65000.0f);
    	_motor->FOC.FOCAngle = temp_angle;
        OLGenerateAngle(_motor);
        temp_angle = _motor->FOC.FOCAngle;
        if(cycles==60001){
        	temp_flux = sqrtf(_motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q)/(6.28f * (float)_motor->FOC.openloop_step * (float)_motor->FOC.pwm_frequency/65536.0f);
        	motor.motor_flux  = temp_flux;
        	_motor->FOC.flux_linked_alpha = _motor->FOC.sincosangle.cos*motor.motor_flux;
        	_motor->FOC.flux_linked_beta = _motor->FOC.sincosangle.sin*motor.motor_flux;
        	motor_profile->flux_linkage_max = 1.7f*motor.motor_flux;
        	motor_profile->flux_linkage_min = 0.5f*motor.motor_flux;
        	temp_FLA = _motor->FOC.flux_linked_alpha;
        	temp_FLB = _motor->FOC.flux_linked_beta;
        }
        MESCFOC(_motor);
    }
    else if (cycles < 61000) {
    	generateBreak(_motor);
    	ADCPhaseConversion(_motor);
    	MESCTrack(_motor);
    }
    else if (cycles < 70001) {
    	generateEnable(_motor);
        _motor->FOC.Idq_int_err.d = 0.0f;
        MESCFOC(_motor);

      count++;
      _motor->FOC.Idq_req.d = 0.0f;
      _motor->FOC.Idq_req.q = IMEASURE_CLOSEDLOOP;
    }
    else if (cycles < 128000) {
      count++;
      _motor->FOC.Idq_req.d = 0.0f;
      _motor->FOC.Idq_req.q = IMEASURE_CLOSEDLOOP;
      MESCFOC(_motor);
    } else {
       generateBreak(_motor);
    	motor_profile->flux_linkage_max = 1.3f*motor.motor_flux;
    	motor_profile->flux_linkage_min = 0.7f*motor.motor_flux;
    	motor_profile->flux_linkage = motor.motor_flux;
      MotorState = MOTOR_STATE_TRACKING;
      cycles = 0;
      if (motor.motor_flux > 0.0001f && motor.motor_flux < 200.0f) {
        MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
      } else {
        MotorState = MOTOR_STATE_ERROR;
        generateBreak(_motor);
      }
    }
    writePWM(_motor);

    cycles++;

  }

  uint32_t tmpccmrx;  // Temporary buffer which is used to turn on/off phase PWMs

  // Turn all phase U FETs off, Tristate the HBridge output - For BLDC mode
  // mainly, but also used for measuring, software fault detection and recovery
  void phU_Break(MESC_motor_typedef *_motor) {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC1M;
    tmpccmrx &= ~TIM_CCMR1_CC1S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC1E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC1NE;  // disable
  }
  // Basically un-break phase U, opposite of above...
  void phU_Enable(MESC_motor_typedef *_motor) {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC1M;
    tmpccmrx &= ~TIM_CCMR1_CC1S;
    tmpccmrx |= TIM_OCMODE_PWM1;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC1E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC1NE;  // enable
  }

  void phV_Break(MESC_motor_typedef *_motor) {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC2M;
    tmpccmrx &= ~TIM_CCMR1_CC2S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE << 8;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC2E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC2NE;  // disable
  }

  void phV_Enable(MESC_motor_typedef *_motor) {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC2M;
    tmpccmrx &= ~TIM_CCMR1_CC2S;
    tmpccmrx |= TIM_OCMODE_PWM1 << 8;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC2E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC2NE;  // enable
  }

  void phW_Break(MESC_motor_typedef *_motor) {
    tmpccmrx = htim1.Instance->CCMR2;
    tmpccmrx &= ~TIM_CCMR2_OC3M;
    tmpccmrx &= ~TIM_CCMR2_CC3S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
    htim1.Instance->CCMR2 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC3E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC3NE;  // disable
  }

  void phW_Enable(MESC_motor_typedef *_motor) {
    tmpccmrx = htim1.Instance->CCMR2;
    tmpccmrx &= ~TIM_CCMR2_OC3M;
    tmpccmrx &= ~TIM_CCMR2_CC3S;
    tmpccmrx |= TIM_OCMODE_PWM1;
    htim1.Instance->CCMR2 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC3E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC3NE;  // enable
  }

#ifndef CURRENT_BANDWIDTH
#define CURRENT_BANDWIDTH 5000.0f
#endif
  void calculateGains(MESC_motor_typedef *_motor) {
    _motor->FOC.pwm_frequency =PWM_FREQUENCY;
    _motor->FOC.pwm_period = 1.0f/_motor->FOC.pwm_frequency;
    htim1.Instance->ARR = HAL_RCC_GetHCLKFreq()/(((float)htim1.Instance->PSC + 1.0f) * 2*_motor->FOC.pwm_frequency);
    htim1.Instance->CCR4 = htim1.Instance->ARR-2; //Just short of dead center (dead center will not actually trigger the conversion)
    #ifdef SINGLE_ADC
    htim1.Instance->CCR4 = htim1.Instance->ARR-50; //If we only have one ADC, we need to convert early otherwise the data will not be ready in time
    #endif
    _motor->FOC.PWMmid = htim1.Instance->ARR * 0.5f;

    _motor->FOC.ADC_duty_threshold = htim1.Instance->ARR * 0.85f;

    _motor->FOC.Id_pgain = CURRENT_BANDWIDTH * motor.Lphase;

    _motor->FOC.Id_igain = motor.Rphase / motor.Lphase;
    // Pole zero cancellation for series PI control

    _motor->FOC.Iq_pgain = _motor->FOC.Id_pgain;
    _motor->FOC.Iq_igain = _motor->FOC.Id_igain;
    _motor->FOC.field_weakening_curr_max = FIELD_WEAKENING_CURRENT;  // test number, to be stored in user settings
  motor.Lqd_diff = motor.Lqphase-motor.Lphase;
  }

  void calculateVoltageGain(MESC_motor_typedef *_motor) {
    // We need a number to convert between Va Vb and raw PWM register values
    // This number should be the bus voltage divided by the ARR register
    _motor->FOC.Vab_to_PWM =
        htim1.Instance->ARR / _motor->Conv.Vbus;
    // We also need a number to set the maximum voltage that can be effectively
    // used by the SVPWM This is equal to
    // 0.5*Vbus*MAX_MODULATION*SVPWM_MULTIPLIER*Vd_MAX_PROPORTION
    _motor->FOC.Vmag_max = 0.5f * _motor->Conv.Vbus *
            MAX_MODULATION * SVPWM_MULTIPLIER;
    _motor->FOC.Vmag_max2 = _motor->FOC.Vmag_max*_motor->FOC.Vmag_max;
    _motor->FOC.Vd_max = 0.5f * _motor->Conv.Vbus *
                      MAX_MODULATION * SVPWM_MULTIPLIER * Vd_MAX_PROPORTION;
    _motor->FOC.Vq_max = 0.5f * _motor->Conv.Vbus *
                      MAX_MODULATION * SVPWM_MULTIPLIER * Vq_MAX_PROPORTION;
#ifdef USE_SQRT_CIRCLE_LIM
    _motor->FOC.Vd_max = _motor->FOC.Vmag_max;
    _motor->FOC.Vq_max = _motor->FOC.Vmag_max;

#endif

    _motor->FOC.Vdint_max = _motor->FOC.Vd_max * 0.9f; //ToDo unvoodoo, logic in this is to always ensure headroom for the P term
    _motor->FOC.Vqint_max = _motor->FOC.Vq_max * 0.9f;

    _motor->FOC.field_weakening_threshold = _motor->FOC.Vq_max * FIELD_WEAKENING_THRESHOLD;
    _motor->FOC.field_weakening_multiplier = 1.0f/(_motor->FOC.Vq_max*(1.0f-FIELD_WEAKENING_THRESHOLD));
  }


  static int dp_periods = 3;
  void doublePulseTest(MESC_motor_typedef *_motor) {
    static int dp_counter;
    if  (dp_counter == 0) { //Let bootstrap charge
        phU_Enable(_motor);
        phV_Enable(_motor);
        phW_Enable(_motor);
        htim1.Instance->CCR1 = 0;
        htim1.Instance->CCR2 = 0;
        htim1.Instance->CCR3 = 0;
        test_vals.dp_current_final[dp_counter] =
            _motor->Conv.Iv;
        dp_counter++;
      } else if(dp_counter <= (dp_periods-2)) { //W State ON
      htim1.Instance->CCR1 = 0;
      htim1.Instance->CCR2 = 0;
      htim1.Instance->CCR3 = htim1.Instance->ARR;
      phU_Break(_motor);
      phV_Enable(_motor);
      phW_Enable(_motor);
      test_vals.dp_current_final[dp_counter] =
          _motor->Conv.Iv;
      dp_counter++;
    } else if (dp_counter == (dp_periods-1)) { //W short second pulse
        htim1.Instance->CCR2 = 0;
        htim1.Instance->CCR3 = 100;
        test_vals.dp_current_final[dp_counter] =
            _motor->Conv.Iv;
        dp_counter++;
     } else if (dp_counter == dp_periods) { //Freewheel a bit to see the current
          htim1.Instance->CCR2 = 0;
          htim1.Instance->CCR3 = 0;
          test_vals.dp_current_final[dp_counter] =
              _motor->Conv.Iv;
          dp_counter++;
        }else { //Turn all off
      htim1.Instance->CCR1 = 0;
      htim1.Instance->CCR2 = 0;
      htim1.Instance->CCR3 = 0;
      test_vals.dp_current_final[dp_counter] =
          _motor->Conv.Iv;
      dp_counter = 0;
      generateBreak(_motor);
      MotorState = MOTOR_STATE_IDLE;
    }
  }
  void MESC_Slow_IRQ_handler(MESC_motor_typedef *_motor){


	  if(_motor->stimer->Instance->SR & TIM_FLAG_CC2){
		  input_vars.IC_duration = _motor->stimer->Instance->CCR1;// HAL_TIM_ReadCapturedValue(&htim4 /*&htim3*/, TIM_CHANNEL_1);
		  input_vars.IC_pulse = _motor->stimer->Instance->CCR2;//HAL_TIM_ReadCapturedValue(&htim4 /*&htim3*/, TIM_CHANNEL_2);
		  input_vars.pulse_recieved = 1;

	  }else{
		  input_vars.IC_duration = 50000;
		  input_vars.IC_pulse = 0;
		  input_vars.pulse_recieved = 0;

	  }

	    if(_motor->stimer->Instance->SR & TIM_FLAG_UPDATE){
	    		      slowLoop(_motor);
	    }
  }
  extern uint32_t ADC_buffer[6];

  void slowLoop(MESC_motor_typedef *_motor) {
    // The slow loop runs at either 20Hz or at the PWM input frequency.
    // In this loop, we will fetch the throttle values, and run functions that
    // are critical, but do not need to be executed very often e.g. adjustment
    // for battery voltage change
if(MotorState != MOTOR_STATE_MEASURING){
	  //Collect the requested throttle inputs
	  //UART input
	  if(0 == (input_vars.input_options & 0b1000)){
		  input_vars.Idq_req_UART.q = 0.0f;
	  }

	  //RCPWM input
	  if(input_vars.input_options & 0b0100){
		  if(input_vars.pulse_recieved){
			  if((input_vars.IC_duration > input_vars.IC_duration_MIN) && (input_vars.IC_duration < input_vars.IC_duration_MAX)){
				  if(input_vars.IC_pulse>(input_vars.IC_pulse_MID + input_vars.IC_pulse_DEADZONE)){
					  input_vars.Idq_req_RCPWM.d = 0.0f;
					  input_vars.Idq_req_RCPWM.q = (float)(input_vars.IC_pulse - (input_vars.IC_pulse_MID + input_vars.IC_pulse_DEADZONE))*input_vars.RCPWM_gain[0][1];
				  }
				  else if(input_vars.IC_pulse<(input_vars.IC_pulse_MID - input_vars.IC_pulse_DEADZONE)){
					  input_vars.Idq_req_RCPWM.d = 0.0f;
					  input_vars.Idq_req_RCPWM.q = ((float)input_vars.IC_pulse - (float)(input_vars.IC_pulse_MID - input_vars.IC_pulse_DEADZONE))*input_vars.RCPWM_gain[0][1];
				  }
				  else{
					  input_vars.Idq_req_RCPWM.d = 0.0f;
					  input_vars.Idq_req_RCPWM.q = 0.0f;
				  }
			  }	else {//The duration of the IC was wrong; trap it and write no current request
				  //Todo maybe want to implement a timeout on this, allowing spurious pulses to not wiggle the current?
				  input_vars.Idq_req_RCPWM.d = 0.0f;
				  input_vars.Idq_req_RCPWM.q = 0.0f;
			  }
		  }
		  else {//No pulse received flag
			  input_vars.Idq_req_RCPWM.d = 0.0f;
			  input_vars.Idq_req_RCPWM.q = 0.0f;
		  }
	  }

	  //ADC1 input
	  if(input_vars.input_options & 0b0010){
		  if(_motor->Raw.ADC_in_ext1>input_vars.adc1_MIN){
			  input_vars.Idq_req_ADC1.d = 0.0f;
			  input_vars.Idq_req_ADC1.q = ((float)_motor->Raw.ADC_in_ext1-(float)input_vars.adc1_MIN)*input_vars.adc1_gain[1]*input_vars.ADC1_polarity;
		  }
		  else{
			  input_vars.Idq_req_ADC1.d = 0.0f;
			  input_vars.Idq_req_ADC1.q = 0.0f;
		  }
	  }
	  if(input_vars.input_options & 0b0001){
		  //ADC2 input
		  //placeholder
	  }

_motor->FOC.Idq_req.q = input_vars.Idq_req_UART.q + input_vars.Idq_req_RCPWM.q + input_vars.Idq_req_ADC1.q + input_vars.Idq_req_ADC2.q;

///////////// Clamp the overall request
if(_motor->FOC.Idq_req.d>input_vars.max_request_Idq.d){_motor->FOC.Idq_req.d = input_vars.max_request_Idq.d;}
if(_motor->FOC.Idq_req.d<input_vars.min_request_Idq.d){_motor->FOC.Idq_req.d = input_vars.min_request_Idq.d;}
if(_motor->FOC.Idq_req.q>input_vars.max_request_Idq.q){_motor->FOC.Idq_req.q = input_vars.max_request_Idq.q;}
if(_motor->FOC.Idq_req.q<input_vars.min_request_Idq.q){_motor->FOC.Idq_req.q = input_vars.min_request_Idq.q;}

////// Adjust the SVPWM gains to account for the change in battery voltage etc
    calculateVoltageGain(_motor);

////// Calculate the current power
    _motor->FOC.currentPower.d = 1.5f*(_motor->FOC.Vdq.d*_motor->FOC.Idq_smoothed.d);
    _motor->FOC.currentPower.q = 1.5f*(_motor->FOC.Vdq.q*_motor->FOC.Idq_smoothed.q);
    //_motor->FOC.currentPowerab = _motor->FOC.Vab[0]*_motor->FOC.Iab[0] + _motor->FOC.Vab[1]*_motor->FOC.Iab[1];
    _motor->FOC.Ibus = (_motor->FOC.currentPower.d + _motor->FOC.currentPower.q) /measurement_buffers.ConvertedADC[0][1];

//Run MTPA (Field weakening seems to have to go in  the fast loop to be stable)
#ifdef USE_MTPA

    if(motor.Lqd_diff>0){
    	_motor->FOC.id_mtpa = motor.motor_flux/(4.0f*motor.Lqd_diff) - sqrtf((motor.motor_flux*motor.motor_flux/(16.0f*motor.Lqd_diff*motor.Lqd_diff))+_motor->FOC.Idq_req.q*_motor->FOC.Idq_req.q*0.5f);
    	if(fabsf(_motor->FOC.Idq_req.q)>fabsf(_motor->FOC.id_mtpa)){
    	_motor->FOC.iq_mtpa = sqrtf(_motor->FOC.Idq_req.q*_motor->FOC.Idq_req.q-_motor->FOC.id_mtpa*_motor->FOC.id_mtpa);
    	}
    	else{
    		_motor->FOC.iq_mtpa = 0;
    	}
    _motor->FOC.Idq_req.d = _motor->FOC.id_mtpa;
    if(_motor->FOC.Idq_req.q>0.0f){
    _motor->FOC.Idq_req.q = _motor->FOC.iq_mtpa;}
    else{
    	_motor->FOC.Idq_req.q = -_motor->FOC.iq_mtpa;}
    }


#endif


    if(getHallState()==0){//This happens when the hall sensors overheat it seems.
  	  if (MotorError == MOTOR_ERROR_NONE) {
  		    speed_motor_limiter();
  	  }
  	  MotorError = MOTOR_ERROR_HALL0;
    }else /*if(getHallState()==7){
  	  MotorError = MOTOR_ERROR_HALL7;
    } else */{
  	  if (MotorError != MOTOR_ERROR_NONE) {
  		  // TODO speed_road();
  	  }
  	  MotorError = MOTOR_ERROR_NONE;
    }

    if(temp_check( measurement_buffers.RawADC[3][0] ) == false){
    	if(0){generateBreak(_motor); //ToDo Currently not loading the profile so commented out - no temp safety!
    	MotorState = MOTOR_STATE_ERROR;
    	MotorError = MOTOR_ERROR_OVER_LIMIT_TEMP;
    	}
    }
/////// Clamp the max power taken from the battery
    _motor->FOC.reqPower = 1.5f*fabsf(_motor->FOC.Vdq.q * _motor->FOC.Idq_req.q);
    if (_motor->FOC.reqPower > motor_profile->Pmax) {
    	if(_motor->FOC.Idq_req.q > 0.0f){
    		_motor->FOC.Idq_req.q = motor_profile->Pmax / (fabsf(_motor->FOC.Vdq.q)*1.5f);
    	}else{
    		_motor->FOC.Idq_req.q = -motor_profile->Pmax / (fabsf(_motor->FOC.Vdq.q)*1.5f);
    	}
    }

////// Unpuc the observer kludge
// The observer gets into a bit of a state if it gets close to
// flux linked = 0 for both accumuators, the angle rapidly changes
// as it oscillates around zero. Solution... just kludge it back out.
// This only happens at stationary when it is useless anyway.
    if ((_motor->FOC.flux_linked_alpha * _motor->FOC.flux_linked_alpha + _motor->FOC.flux_linked_beta * _motor->FOC.flux_linked_beta) <
        0.25f * motor.motor_flux * motor.motor_flux) {
    	_motor->FOC.flux_linked_alpha = 0.5f * motor.motor_flux;
    	_motor->FOC.flux_linked_beta = 0.5f * motor.motor_flux;
    }

//////////////////////Run the LR observer
#ifdef USE_LR_OBSERVER
    LRObserver();
#endif
    //////Set tracking
static int was_last_tracking;

if(!((MotorState==MOTOR_STATE_MEASURING)||(MotorState==MOTOR_STATE_DETECTING)||(MotorState==MOTOR_STATE_GET_KV)||(MotorState==MOTOR_STATE_TEST)||(MotorState==MOTOR_STATE_INITIALISING)||(MotorState==MOTOR_STATE_SLAMBRAKE))){
	if(fabsf(_motor->FOC.Idq_req.q)>0.1f){

		if(MotorState != MOTOR_STATE_ERROR){
	#ifdef HAS_PHASE_SENSORS //We can go straight to RUN if we have been tracking with phase sensors
		MotorState = MOTOR_STATE_RUN;
		//generateEnable();
	#endif
	if(MotorState==MOTOR_STATE_IDLE){
	#ifdef USE_DEADSHORT
		MotorState = MOTOR_STATE_RECOVERING;

	#endif
			}
		}
	}else if(FW_current>-0.5f){	//Keep it running if FW current is being used
	#ifdef HAS_PHASE_SENSORS
		MotorState = MOTOR_STATE_TRACKING;
		VICheck(_motor); //Immediately return it to error state if there is still a critical fault condition active
	#else
	//	if(MotorState != MOTOR_STATE_ERROR){
		MotorState = MOTOR_STATE_IDLE;
	//	}
	#endif
	was_last_tracking = 1;
	}else{	//Ramp down the field weakening current
			//Do NOT assign motorState here, since it could override error states
		FW_current*=0.95f;
		if(_motor->FOC.Vdq.q <0.0f){
			_motor->FOC.Idq_req.q = 0.2f; //Apply a brake current
		}
		if(_motor->FOC.Vdq.q >0.0f){
			_motor->FOC.Idq_req.q = -0.2f; //Apply a brake current
		}
	}
}
//_motor->FOC.Idq_req[0] = 10; //for aligning encoder
/////////////Set and reset the HFI////////////////////////
#ifdef USE_HFI
    if((_motor->FOC.Vdq.q > 2.0f)||(_motor->FOC.Vdq.q < -2.0f)||(MotorSensorMode==MOTOR_SENSOR_MODE_HALL)){
    	_motor->FOC.inject = 0;
    } else if((_motor->FOC.Vdq.q < 1.0f)&&(_motor->FOC.Vdq.q > -1.0f)){
    	_motor->FOC.inject = 1;
  	  _motor->FOC.Vd_injectionV = HFI_VOLTAGE;
  	  _motor->FOC.Vq_injectionV = 0.0f;
    }

    if(_motor->FOC.inject==1){
    	static int HFI_countdown;
    	static int no_q;
    	if(was_last_tracking==1){
    		HFI_countdown = 4; //resolve the ambiguity immediately
    		_motor->FOC.Idq_req.q = 0.0f;
    		no_q=1;
    		was_last_tracking = 0;
    	}
		if(HFI_countdown==3){
			_motor->FOC.Idq_req.d = HFI_TEST_CURRENT;
		}else if(HFI_countdown==2){
			_motor->FOC.Ldq_now_dboost[0] = _motor->FOC.IIR[0]; //Find the effect of d-axis current
			_motor->FOC.Idq_req.d = 1.0f;
		}else if(HFI_countdown == 1){
			_motor->FOC.Idq_req.d = -50.0f;
		}else if(HFI_countdown == 0){
			_motor->FOC.Ldq_now[0] = _motor->FOC.IIR[0];//_motor->FOC.Vd_injectionV;
			_motor->FOC.Idq_req.d = 1.0f;
		if(_motor->FOC.Ldq_now[0]>_motor->FOC.Ldq_now_dboost[0]){_motor->FOC.FOCAngle+=32768;}
		HFI_countdown = 200;
		no_q = 0;
		}else{
			_motor->FOC.Idq_req.d = 0.0f;
		}
		HFI_countdown--;
	    if(no_q){_motor->FOC.Idq_req.q=0.0f;}
    }
#else
    _motor->FOC.inject = 0;
#endif

    //Speed tracker
    if(abs(_motor->FOC.angle_error)>6000){
    	_motor->FOC.angle_error = 0;
    }
  }
  }


  void MESCTrack(MESC_motor_typedef *_motor) {
    // here we are going to do the clark and park transform of the voltages to
    // get the VaVb and VdVq These can be handed later to the observers and used
    // to set the integral terms

    // Clark transform
    _motor->FOC.Vab[0] =
        0.666f * (_motor->Conv.Vu -
                  0.5f * ((_motor->Conv.Vv) +
                          (_motor->Conv.Vw)));
    _motor->FOC.Vab[1] =
        0.666f *
        (sqrt3_on_2 * ((_motor->Conv.Vv) -
                       (_motor->Conv.Vw)));

    sin_cos_fast(_motor->FOC.FOCAngle, &_motor->FOC.sincosangle.sin, &_motor->FOC.sincosangle.cos);

    // Park transform

    _motor->FOC.Vdq.d = _motor->FOC.sincosangle.cos * _motor->FOC.Vab[0] +
                      _motor->FOC.sincosangle.sin * _motor->FOC.Vab[1];
    _motor->FOC.Vdq.q = _motor->FOC.sincosangle.cos * _motor->FOC.Vab[1] -
                      _motor->FOC.sincosangle.sin * _motor->FOC.Vab[0];
    _motor->FOC.Idq_int_err.q = _motor->FOC.Vdq.q;
  }


  float IacalcDS, IbcalcDS, VacalcDS, VbcalcDS, VdcalcDS, VqcalcDS, FLaDS, FLbDS, FLaDSErr, FLbDSErr;
  uint16_t angleDS, angleErrorDSENC, angleErrorPhaseSENC, angleErrorPhaseDS, countdown_cycles;

  void deadshort(MESC_motor_typedef *_motor){
	  // LICENCE NOTE:
	  // This function deviates slightly from the BSD 3 clause licence.
	  // The work here is entirely original to the MESC FOC project, and not based
	  // on any appnotes, or borrowed from another project. This work is free to
	  // use, as granted in BSD 3 clause, with the exception that this note must
	  // be included in where this code is implemented/modified to use your
	  // variable names, structures containing variables or other minor
	  // rearrangements in place of the original names I have chosen, and credit
	  // to David Molony as the original author must be noted.

	  //This "deadshort " function is an original idea (who knows, someone may have had it before) for finding the rotor angle
	  //Concept is that when starting from spinning with no phase sensors or encoder, you need to know the angle and the voltages.
	  //To achieve this, we simply short out the motor for a PWM period and allow the current to build up.
	  //We can then calculate the voltage from V=Ldi/dt in the alpha beta reference frame
	  //We can calculate the angle from the atan2 of the alpha beta voltages
	  //With this angle, we can get Vd and Vq for preloading the PI controllers
	  //We can also preload the flux observer with motor.motorflux*sin and motor.motorflux*cos terms

	static uint16_t countdown = 10;

	  		if(countdown == 1||(((_motor->FOC.Iab[0]*_motor->FOC.Iab[0]+_motor->FOC.Iab[1]*_motor->FOC.Iab[1])>DEADSHORT_CURRENT*DEADSHORT_CURRENT)&&countdown<9))
	  				{
	  					//Need to collect the ADC currents here
	  					generateBreak(_motor);
	  					//Calculate the voltages in the alpha beta phase...
	  					IacalcDS = _motor->FOC.Iab[0];
	  					IbcalcDS = _motor->FOC.Iab[1];
	  					VacalcDS = -motor.Lphase*_motor->FOC.Iab[0]/((9.0f-(float)countdown)*_motor->FOC.pwm_period);
	  					VbcalcDS = -motor.Lphase*_motor->FOC.Iab[1]/((9.0f-(float)countdown)*_motor->FOC.pwm_period);
	  					//Calculate the phase angle
	  					//TEST LINE angleDS = (uint16_t)(32768.0f + 10430.0f * fast_atan2(VbcalcDS, VacalcDS)) - 32768;// +16384;

	  					 angleDS = (uint16_t)(32768.0f + 10430.0f * fast_atan2(VbcalcDS, VacalcDS)) - 32768 -16384;
	  					//Shifting by 1/4 erev does not work for going backwards. Need to rethink.
	  					//Problem is, depending on motor direction, the sign of the voltage generated swaps for the same rotor position.
	  					//The atan2(flux linkages) is stable under this regime, but the same for voltage is not.
	  					_motor->FOC.FOCAngle = angleDS;//_motor->FOC.enc_angle;//
	  					sin_cos_fast(_motor->FOC.FOCAngle, &_motor->FOC.sincosangle.sin, &_motor->FOC.sincosangle.cos);

	  					//Park transform it to get VdVq
	  					VdcalcDS = _motor->FOC.sincosangle.cos * VacalcDS +
	  				                      _motor->FOC.sincosangle.sin * VbcalcDS;
	  					VqcalcDS = _motor->FOC.sincosangle.cos * VbcalcDS -
	  				                      _motor->FOC.sincosangle.sin * VacalcDS;
	  					//Preloading the observer
	  					FLaDS = motor.motor_flux*_motor->FOC.sincosangle.cos;
	  					FLbDS = motor.motor_flux*_motor->FOC.sincosangle.sin;
	  		//Angle Errors for debugging
	  					angleErrorDSENC = angleDS-_motor->FOC.enc_angle;
	  		//			angleErrorPhaseSENC = _motor->FOC.FOCAngle-_motor->FOC.enc_angle;
	  		//			angleErrorPhaseDS = _motor->FOC.FOCAngle - angleDS;
	  		//Variables for monitoring and debugging to see if the preload will work
	  		//			FLaDSErr = 1000.0f*(FLaDS-_motor->FOC.flux_linked_alpha);
	  		//			FLbDSErr = 1000.0f*(FLbDS-_motor->FOC.flux_linked_beta);

	  		//Do actual preloading
	  					_motor->FOC.flux_linked_alpha = FLaDS;
	  					_motor->FOC.flux_linked_beta = FLbDS;
	  					Ia_last = 0.0f;
	  					Ib_last = 0.0f;
	  					_motor->FOC.Idq_int_err.d = VdcalcDS;
	  					_motor->FOC.Idq_int_err.q = VqcalcDS;
	  		//Next PWM cycle it  will jump to running state,
	  					MESCFOC(_motor);
	  					countdown_cycles = 9-countdown;
	  					countdown = 1;



	  		}
	  		if(countdown > 10){
	  			generateBreak(_motor);
	  			htim1.Instance->CCR1 = 50;
	  			htim1.Instance->CCR2 = 50;
	  			htim1.Instance->CCR3 = 50;
	  			//Preload the timer at mid
	  		}
	  		if(countdown <= 10 && countdown>1 ){
	  			htim1.Instance->CCR1 = 50;
	  			htim1.Instance->CCR2 = 50;
	  			htim1.Instance->CCR3 = 50;
	  			generateEnable(_motor);
	  		}
	  		if(countdown == 1 ){
					countdown = 15; //We need at least a few cycles for the current to relax
									//to zero in case of rapid switching between states
  					MotorState = MOTOR_STATE_RUN;

	  		}
	  		countdown--;
  }

  uint8_t pkt_crc8(uint8_t crc/*CRC_SEED=0xFF*/, uint8_t *data, uint8_t length)
  {
      int16_t i, bit;

      for (i = 0; i < length; i++)
      {
          crc ^= data[i];

          for (bit = 0; bit < 8; bit++)
          {
              if ((crc & 0x80) != 0)
              {
                  crc <<= 1;
                  crc ^= 0x1D; //CRC_POLYNOMIAL=0x1D;
              }
              else
              {
                  crc <<= 1;
              }
          }
      }

      return crc;
  }

  struct __attribute__ ((__packed__))SamplePacket
  {
	  	struct
	  	{
	  		uint8_t crc;
	  		uint8_t STAT_RESP; // Should be 0xF_?
	  	}safetyword;
  	uint16_t angle;
  	int16_t speed;
  	uint16_t revolutions;
  };

  typedef struct SamplePacket SamplePacket;
	  SamplePacket pkt;

  void tle5012(void)
  {
#ifdef USE_ENCODER
	  uint16_t const len = sizeof(pkt) / sizeof(uint16_t);
	  uint16_t reg = (UINT16_C(  1) << 15) /* RW=Read */
	               | (UINT16_C(0x0) << 11) /* Lock */
	               | (UINT16_C(0x0) << 10) /* UPD=Buffer */
	               | (UINT16_C(0x02) << 4) /* ADDR */
	               | (len -1);            /* ND */
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
      HAL_SPI_Transmit( &hspi3, (uint8_t *)&reg,   1, 1000 );
      HAL_SPI_Receive(  &hspi3, (uint8_t *)&pkt, len, 1000 );
//      volatile uint8_t crc = 0;
//#if 1
//      reg ^= 0xFF00;
//      crc = pkt_crc8( crc, &((uint8_t *)&reg)[1], 1 );
//      crc = pkt_crc8( crc, &((uint8_t *)&reg)[0], 1 );
//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.angle)[1], 1 );
//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.angle)[0], 1 );
//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.speed)[1], 1 );
//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.speed)[0], 1 );
//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.revolutions)[1], 1 );
//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.revolutions)[0], 1 );
//#else
//      crc = pkt_crc8( crc, &reg, 2 );
//      crc = pkt_crc8( crc, &pkt.angle, 6 );
//#endif
//      crc = pkt_crc8( crc, &pkt.safetyword.STAT_RESP, 1 );
//      crc = ~crc;
//      if (crc != pkt.safetyword.crc)
//      {
//    	  __NOP();
//    	  __NOP();
//    	  __NOP();
//      }
//      else
//      {
//    	  __NOP();
//      }

      pkt.angle = pkt.angle & 0x7fff;
#ifdef ENCODER_DIR_REVERSED
      	  _motor->FOC.enc_angle = -POLE_PAIRS*((pkt.angle *2)%POLE_ANGLE)-_motor->FOC.enc_offset;
#else
      _motor->FOC.enc_angle = POLE_PAIRS*((pkt.angle *2)%POLE_ANGLE)-_motor->FOC.enc_offset;
#endif
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
      pkt.revolutions = pkt.revolutions&0b0000000111111111;
#endif
  }


uint16_t test_on_time;
uint16_t test_on_time_acc[3];
uint16_t test_counts;
  void getDeadtime(MESC_motor_typedef *_motor){
	  static int use_phase = 0;

	  if(test_on_time<1){test_on_time = 1;}

		if(use_phase==0){
			htim1.Instance->CCR1 = test_on_time;
			htim1.Instance->CCR2 = 0;
			htim1.Instance->CCR3 = 0;
			if(_motor->Conv.Iu<1.0f){ test_on_time=test_on_time+1;}
			if(_motor->Conv.Iu>1.0f){ test_on_time=test_on_time-1;}
			generateEnable(_motor);
			test_on_time_acc[0] = test_on_time_acc[0]+test_on_time;
			}
		if(use_phase==1){
			htim1.Instance->CCR1 = 0;
			htim1.Instance->CCR2 = test_on_time;
			htim1.Instance->CCR3 = 0;
			if(_motor->Conv.Iv<1.0f){ test_on_time=test_on_time+1;}
			if(_motor->Conv.Iv>1.0f){ test_on_time=test_on_time-1;}
			generateEnable(_motor);
			test_on_time_acc[1] = test_on_time_acc[1]+test_on_time;
		}
		if(use_phase==2){
			htim1.Instance->CCR1 = 0;
			htim1.Instance->CCR2 = 0;
			htim1.Instance->CCR3 = test_on_time;
			if(_motor->Conv.Iw<1.0f){ test_on_time=test_on_time+1;}
			if(_motor->Conv.Iw>1.0f){ test_on_time=test_on_time-1;}
			generateEnable(_motor);
			test_on_time_acc[2] = test_on_time_acc[2]+test_on_time;
		}
		if(use_phase>2){
			generateBreak(_motor);
			MotorState = MOTOR_STATE_TRACKING;
			use_phase = 0;
			test_on_time_acc[0] = test_on_time_acc[0]>>10;
			test_on_time_acc[1] = test_on_time_acc[1]>>10;
			test_on_time_acc[2] = test_on_time_acc[2]>>10;
			deadtime_comp = test_on_time_acc[0];
		}
		test_counts++;

	if(test_counts>511){
		use_phase++;
	test_counts = 0;
	}
  }

  //LR observer. WIP function that works by injecting a
  //low frequency Id signal into the PID input and observing the change in Vd and Vq
  //Does not work too well, requires some care in use.
  //Original work to MESC project.
  float Vd_obs_high, Vd_obs_low, R_observer, Vq_obs_high, Vq_obs_low, L_observer, Last_eHz, LR_collect_count;
  float Vd_obs_high_filt, Vd_obs_low_filt,Vq_obs_high_filt,Vq_obs_low_filt;
  static int plusminus = 1;

  void LRObserver(MESC_motor_typedef *_motor){
	  if((fabs(_motor->FOC.eHz)>0.005*_motor->FOC.pwm_frequency)&&(_motor->FOC.inject ==0)){




	  R_observer = (Vd_obs_high_filt-Vd_obs_low_filt)/(2.0f*LR_OBS_CURRENT);
	  L_observer = (Vq_obs_high_filt-Vq_obs_low_filt-6.28f*(_motor->FOC.eHz-Last_eHz)*motor.motor_flux)/(2.0f*LR_OBS_CURRENT*6.28f*_motor->FOC.eHz);

	  	if(plusminus==1){
	  		plusminus = -1;
	  	  Vd_obs_low_filt = Vd_obs_low/LR_collect_count;
	  	  Vq_obs_low_filt = Vq_obs_low/LR_collect_count;
	  	  _motor->FOC.Idq_req.d = _motor->FOC.Idq_req.d+1.0f*LR_OBS_CURRENT;
  		  Vd_obs_low = 0;
  		  Vq_obs_low = 0;
	  	}else if(plusminus == -1){
	  		plusminus = 1;
	  	  Vd_obs_high_filt = Vd_obs_high/LR_collect_count;
	  	  Vq_obs_high_filt = Vq_obs_high/LR_collect_count;
	  	  _motor->FOC.Idq_req.d = _motor->FOC.Idq_req.d-1.0f*LR_OBS_CURRENT;
  		  Vd_obs_high = 0;
  		  Vq_obs_high = 0;
	  	}
	  	Last_eHz = _motor->FOC.eHz;
		  LR_collect_count = 0; //Reset this after doing the calcs


	  }
#if 0
	  	float Rerror = R_observer-motor.Rphase;
	  	float Lerror = L_observer-motor.Lphase;
	  	//Apply the correction excluding large changes
	  	if(fabs(Rerror)<0.1f*motor.Rphase){
	  		motor.Rphase = motor.Rphase+0.1f*Rerror;
	  	}else if(fabs(Rerror)<0.5f*motor.Rphase){
	  		motor.Rphase = motor.Rphase+0.001f*Rerror;
	  	}
	  	if(fabs(Lerror)<0.1f*motor.Lphase){
	  		motor.Lphase = motor.Lphase+0.1f*Lerror;
	  		motor.Lqphase = motor.Lqphase +0.1f*Lerror;
	  	}else if(fabs(Lerror)<0.5f*motor.Lphase){
	  		motor.Lphase = motor.Lphase+0.001f*Lerror;
	  		motor.Lqphase = motor.Lqphase +0.001f*Lerror;
	  	}

#endif
  }

  void LRObserverCollect(MESC_motor_typedef *_motor){
	  LR_collect_count++;
	  if((fabs(_motor->FOC.eHz)>0.005*_motor->FOC.pwm_frequency)&&(_motor->FOC.inject ==0)){
	  	  	if(plusminus==1){
	  	  		Vd_obs_low = Vd_obs_low + _motor->FOC.Vdq.d;
	  	  		Vq_obs_low = Vq_obs_low + _motor->FOC.Vdq.q;
	  	  	}
	  	  	if(plusminus == -1){
	  	  		Vd_obs_high = Vd_obs_high + _motor->FOC.Vdq.d;
	  	  		Vq_obs_high = Vq_obs_high + _motor->FOC.Vdq.q;
	  	  	}
	  }
  }

  void HallFluxMonitor(MESC_motor_typedef *_motor){
	  if(fabs(_motor->FOC.Vdq.q)>10.0f){ //Are we actually spinning at a reasonable pace?
		  if((current_hall_state>0)&&(current_hall_state<7)){
	  _motor->FOC.hall_flux[current_hall_state - 1][0] =
			  0.999f*_motor->FOC.hall_flux[current_hall_state - 1][0] +
			  0.001f*_motor->FOC.flux_linked_alpha;
	  //take a slow average of the alpha flux linked and store it for later preloading
	  //the observer during very low speed conditions. There is a slight bias towards
	  //later values of flux linked, which is probably good.
	  _motor->FOC.hall_flux[current_hall_state - 1][1] =
			  0.999f*_motor->FOC.hall_flux[current_hall_state - 1][1] +
			  0.001f*_motor->FOC.flux_linked_beta;
		  }
		  _motor->FOC.hall_initialised = 1;
	  }
  }


void  logVars(MESC_motor_typedef *_motor){
	sampled_vars.Vbus[sampled_vars.current_sample] = _motor->Conv.Vbus;
	sampled_vars.Iu[sampled_vars.current_sample] = _motor->Conv.Iu;
	sampled_vars.Iv[sampled_vars.current_sample] = _motor->Conv.Iv;
	sampled_vars.Iw[sampled_vars.current_sample] = _motor->Conv.Iw;
	sampled_vars.Vd[sampled_vars.current_sample] = _motor->FOC.Vdq.d;
	sampled_vars.Vq[sampled_vars.current_sample] = _motor->FOC.Vdq.q;
	sampled_vars.angle[sampled_vars.current_sample] = _motor->FOC.FOCAngle;
	sampled_vars.current_sample++;
	if(sampled_vars.current_sample>=LOGLENGTH){
		sampled_vars.current_sample = 0;
	}
}

int samples_sent;
uint32_t start_ticks;
extern DMA_HandleTypeDef hdma_usart3_tx;
void printSamples(UART_HandleTypeDef *uart, DMA_HandleTypeDef *dma){
	char send_buffer[100];
	uint16_t length;
#ifdef LOGGING
	if(print_samples_now){
		print_samples_now = 0;
		lognow = 0;
		int current_sample_pos = sampled_vars.current_sample;
		start_ticks = HAL_GetTick();

		samples_sent = 0;

		while(samples_sent<LOGLENGTH){
				HAL_Delay(1);//Wait 2ms, would be nice if we could poll for the CDC being free...
				samples_sent++;
				current_sample_pos++;

				if(current_sample_pos>=LOGLENGTH){
					current_sample_pos = 0;	//Wrap
				}

				length = sprintf(send_buffer,"%.2f%.2f,%.2f,%.2f,%.2f,%.2f,%d;\r\n",
						sampled_vars.Vbus[current_sample_pos],
						sampled_vars.Iu[current_sample_pos],
						sampled_vars.Iv[current_sample_pos],
						sampled_vars.Iw[current_sample_pos],
						sampled_vars.Vd[current_sample_pos],
						sampled_vars.Vq[current_sample_pos],
						sampled_vars.angle[current_sample_pos]);
				if((HAL_GetTick()-start_ticks)>10000){
				break;
				}
#ifdef MESC_UART_USB
		CDC_Transmit_FS(send_buffer, length);
#else

		HAL_UART_Transmit_DMA(uart, send_buffer, length);
			while(hdma_usart3_tx.State != HAL_DMA_STATE_READY){//Pause here
		//	while(hdma_usart3_tx.Lock != HAL_UNLOCKED){//Pause here
				__NOP();
			}
#endif
		}
		lognow = 1;
	}
#endif
}

  // clang-format on
