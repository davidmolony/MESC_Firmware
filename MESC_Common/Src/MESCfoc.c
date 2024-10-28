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

#include "MESChw_setup.h"
#include "MESCmotor_state.h"
#include "MESCsin_lut.h"
#include "MESCmotor.h"
#include "MESCtemp.h"
#include "MESCerror.h"
#include "MESCposition.h"
#include "MESChfi.h"
#include "MESCpwm.h"
#include "MESCinput.h"
#include "MESCmeasure.h"
#include "MESCfluxobs.h"
#include "MESClrobs.h"
#include "MESCBLDC.h"
#include "MESCApp.h"

#include "conversions.h"

#include <math.h>
#include <stdlib.h>
#ifdef LOGGING
#include <stdio.h>
#endif

extern TIM_HandleTypeDef htim4;

float one_on_sqrt3 = 0.577350f;
float one_on_sqrt2 = 0.707107f;
float sqrt2 = 1.41421f;
float sqrt1_2 = 0.707107f;
float sqrt3_on_2 = 0.866025f;
float two_on_sqrt3 = 1.15470f;

MESC_motor_typedef mtr[NUM_MOTORS];

extern ADC_HandleTypeDef hadc1;


//Debug
#define DEMCR_TRCENA    0x01000000
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT

static void SlowStartup(MESC_motor_typedef *_motor);
static void calculatePower(MESC_motor_typedef *_motor);
static void LimitFWCurrent(MESC_motor_typedef *_motor);
static void houseKeeping(MESC_motor_typedef *_motor);
static void clampBatteryPower(MESC_motor_typedef *_motor);
static void ThrottleTemperature(MESC_motor_typedef *_motor);
static void FWRampDown(MESC_motor_typedef *_motor);

void MESCfoc_Init(MESC_motor_typedef *_motor) {
#ifdef STM32L4 // For some reason, ST have decided to have a different name for the L4 timer DBG freeze...
	DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP;
#else
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
#endif
#ifdef FASTLED
	FASTLED->MODER |= 0x1<<(FASTLEDIONO*2);
	FASTLED->MODER &= ~(0x2<<(FASTLEDIONO*2));
#endif
#ifdef SLOWLED
	SLOWLED->MODER |= 0x1<<(SLOWLEDIONO*2);
	SLOWLED->MODER &= ~(0x2<<(SLOWLEDIONO*2));
#endif

#ifdef KILLSWITCH_GPIO
	KILLSWITCH_GPIO->MODER &= ~(0b11<<(2*KILLSWITCH_IONO));
#endif

#ifdef HANDBRAKE_GPIO
	HANDBRAKE_GPIO->MODER &= ~(0b11<<(2*HANDBRAKE_IONO));
#endif

#ifdef BRAKE_DIGITAL_GPIO
	BRAKE_DIGITAL_GPIO->MODER &= ~(0b11<<(2*BRAKE_DIGITAL_IONO));
#endif

#ifdef INV_ENABLE_M1
	INV_ENABLE_M1->MODER |= 0x1<<(INV_ENABLE_M1_IONO*2);
	INV_ENABLE_M1->MODER &= ~(0x2<<(INV_ENABLE_M1_IONO*2));
#endif
#ifdef INV_ENABLE_M2
	INV_ENABLE_M2->MODER |= 0x1<<(INV_ENABLE_M2_IONO*2);
	INV_ENABLE_M2->MODER &= ~(0x2<<(INV_ENABLE_M2_IONO*2));
#endif
	_motor->safe_start[0] = SAFE_START_DEFAULT;


	_motor->MotorState = MOTOR_STATE_IDLE;

	//enable cycle counter
	DEMCR |= DEMCR_TRCENA;
	DWT_CTRL |= CYCCNTENA;

	_motor->offset.Iu = ADC_OFFSET_DEFAULT;
	_motor->offset.Iv = ADC_OFFSET_DEFAULT;
	_motor->offset.Iw = ADC_OFFSET_DEFAULT;

	_motor->FOC.deadtime_comp = DEADTIME_COMP_V;

	_motor->MotorState = MOTOR_STATE_INITIALISING;

	//At this stage, we initialise the options
	_motor->MotorControlType = MOTOR_CONTROL_TYPE_FOC;
	_motor->ControlMode = DEFAULT_CONTROL_MODE;

	_motor->MotorSensorMode = DEFAULT_SENSOR_MODE;
	_motor->HFI.Type = DEFAULT_HFI_TYPE;

	_motor->meas.measure_current = I_MEASURE;
	_motor->meas.measure_voltage = V_MEASURE;
	_motor->meas.measure_closedloop_current = I_MEASURE_CLOSEDLOOP;
	_motor->FOC.pwm_frequency =PWM_FREQUENCY;
	_motor->meas.hfi_voltage = HFI_VOLTAGE;


	//Init Hall sensor
	_motor->hall.dir = 1.0f;
	_motor->hall.ticks_since_last_observer_change = 65535.0f;
	_motor->hall.last_observer_period = 65536.0f;
	_motor->hall.one_on_last_observer_period = 1.0f;
	_motor->hall.angular_velocity = 0.0f;
	_motor->hall.angle_step = 0.0f;

	//options
    //Initialise the hall start
#ifdef USE_HALL_START
	_motor->options.use_hall_start = true;
#else
	_motor->options.use_hall_start = false;
#endif
    _motor->FOC.hall_IIR = HALL_IIR; //decay constant for the hall start preload
    _motor->FOC.hall_IIR = HALL_IIRN; //decay constant for the hall start preload
    _motor->FOC.hall_transition_V = HALL_VOLTAGE_THRESHOLD; //transition voltage above which the hall sensors are not doing any preloading


#ifdef USE_LR_OBSERVER
	_motor->options.use_lr_observer = true;
#else
	_motor->options.use_lr_observer = false;
#endif

#ifdef USE_MTPA
	_motor->options.MTPA_mode = MTPA_MAG;
#else
	_motor->options.MTPA_mode = MTPA_NONE;
#endif

#ifdef USE_HIGHHOPES_PHASE_BALANCING
	_motor->options.use_phase_balancing = true;
#else
	_motor->options.use_phase_balancing = false;
#endif

	_motor->options.field_weakening = FIELD_WEAKENING_OFF;
#ifdef USE_FIELD_WEAKENING
	_motor->options.field_weakening = FIELD_WEAKENING_V1;
#endif

#ifdef USE_FIELD_WEAKENINGV2
	_motor->options.field_weakening = FIELD_WEAKENING_V2;
#endif

	_motor->options.observer_type = MXLEMMING_LAMBDA;
#ifdef USE_ORTEGA_ORIGINAL
	_motor->options.field_weakening = ORTEGA_ORIGINAL;
#endif

	_motor->options.sqrt_circle_lim = SQRT_CIRCLE_LIM_OFF;
#ifdef USE_SQRT_CIRCLE_LIM
	_motor->options.sqrt_circle_lim = SQRT_CIRCLE_LIM_ON;
#endif

#ifdef USE_SQRT_CIRCLE_LIM_VD
	_motor->options.sqrt_circle_lim = SQRT_CIRCLE_LIM_VD;
#endif

	_motor->options.pwm_type = PWM_SVPWM;//Default to combined bottom clamp sinusoidal combinationPWM
#ifdef SIN_BOTTOM
	_motor->options.pwm_type = PWM_SIN_BOTTOM;
#endif

	_motor->options.app_type = APP_NONE;//Default to no app
#ifdef APP_VEHICLE
	_motor->options.app_type = APP_VEHICLE;
#endif


	//PWM Encoder
	_motor->FOC.enc_offset = ENCODER_E_OFFSET;
	_motor->FOC.encoder_polarity_invert = DEFAULT_ENCODER_POLARITY;
	_motor->FOC.enc_period_count = 1; //Avoid /0s

	//ABI Incremental encoder
	_motor->m.enc_counts = 4096;//Default to this, common for many motors. Avoid div0.
	_motor->FOC.enc_ratio = 65536/_motor->m.enc_counts;


	_motor->hall.hall_error = 0;
	//Init the BLDC
	_motor->BLDC.com_flux = _motor->m.flux_linkage*1.65f;//0.02f;
	_motor->BLDC.direction = -1;

	//Init the speed controller
	_motor->FOC.speed_kp = DEFAULT_SPEED_KP; //0.01 = 10A/1000eHz
	_motor->FOC.speed_ki = DEFAULT_SPEED_KI; //Trickier to set since we want this to be proportional to the ramp speed? Not intuitive? Try 0.1; ramp in 1/10 of a second @100Hz.
	//Init the Duty controller
	_motor->FOC.Duty_scaler = 1.0f; //We want this to be 1.0f for everything except duty control mode.
	//Init the PLL values
	_motor->FOC.PLL_kp = PLL_KP;
	_motor->FOC.PLL_ki = PLL_KI;
	//	//Init the POS values
	_motor->pos.Kp = POS_KP;
	_motor->pos.Ki = POS_KI;
	_motor->pos.Kd = POS_KD;
	//
	_motor->Raw.MOS_temp.V                  = 3.3f;
	_motor->Raw.MOS_temp.R_F                = MESC_TEMP_MOS_R_F;
	_motor->Raw.MOS_temp.adc_range          = 4096;
	_motor->Raw.MOS_temp.method             = MESC_TEMP_MOS_METHOD;
	_motor->Raw.MOS_temp.schema             = MESC_TEMP_MOS_SCHEMA;
	_motor->Raw.MOS_temp.parameters.SH.Beta = MESC_TEMP_MOS_SH_BETA;
	_motor->Raw.MOS_temp.parameters.SH.r    = MESC_TEMP_MOS_SH_R;
	_motor->Raw.MOS_temp.parameters.SH.T0   = CVT_CELSIUS_TO_KELVIN_F( 25.0f );
	_motor->Raw.MOS_temp.parameters.SH.R0   = MESC_TEMP_MOS_SH_R0;
	_motor->Raw.MOS_temp.limit.Tmin         = CVT_CELSIUS_TO_KELVIN_F( -15.0f );
	_motor->Raw.MOS_temp.limit.Thot         = CVT_CELSIUS_TO_KELVIN_F(  80.0f );
	_motor->Raw.MOS_temp.limit.Tmax         = CVT_CELSIUS_TO_KELVIN_F( 100.0f );

	_motor->Raw.Motor_temp.V                  = 3.3f;
	_motor->Raw.Motor_temp.R_F                = MESC_TEMP_MOTOR_R_F;
	_motor->Raw.Motor_temp.adc_range          = 4096;
	_motor->Raw.Motor_temp.method             = MESC_TEMP_MOTOR_METHOD;
	_motor->Raw.Motor_temp.schema             = MESC_TEMP_MOTOR_SCHEMA;
	_motor->Raw.Motor_temp.parameters.SH.Beta = MESC_TEMP_MOTOR_SH_BETA;
	_motor->Raw.Motor_temp.parameters.SH.r    = MESC_TEMP_MOTOR_SH_R;
	_motor->Raw.Motor_temp.parameters.SH.T0   = CVT_CELSIUS_TO_KELVIN_F( 25.0f );
	_motor->Raw.Motor_temp.parameters.SH.R0   = MESC_TEMP_MOTOR_SH_R0;
	_motor->Raw.Motor_temp.limit.Tmin         = CVT_CELSIUS_TO_KELVIN_F( -15.0f );
	_motor->Raw.Motor_temp.limit.Thot         = CVT_CELSIUS_TO_KELVIN_F(  80.0f );
	_motor->Raw.Motor_temp.limit.Tmax         = CVT_CELSIUS_TO_KELVIN_F( 100.0f );

	//Initialise the FOC parameters
	//Init the FW
    _motor->FOC.FW_curr_max = FIELD_WEAKENING_CURRENT;  // test number, to be stored in user settings

    //Init the current controller
    _motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH;

    _motor->FOC.ortega_gain = 1000000.0f;

    MESClrobs_Init(_motor);

	mesc_init_1(_motor);

	HAL_Delay(1000);  // Give the everything else time to start up (e.g. throttle,
					// controller, PWM source...)

	mesc_init_2(_motor);

	hw_init(_motor);  // Populate the resistances, gains etc of the PCB - edit within
			  // this function if compiling for other PCBs
//Reconfigure dead times
//This is only useful up to 1500ns for 168MHz clock, 3us for an 84MHz clock
#ifdef CUSTOM_DEADTIME
  uint32_t tempDT;
  uint32_t tmpbdtr = 0U;
  tmpbdtr = mtr->mtimer->Instance->BDTR;
  tempDT = (uint32_t)(((float)CUSTOM_DEADTIME * (float)HAL_RCC_GetHCLKFreq())/(float)1000000000.0f);
  if(tempDT<128){
  MODIFY_REG(tmpbdtr, TIM_BDTR_DTG, tempDT);
  }else{
	  uint32_t deadtime = CUSTOM_DEADTIME;
	  deadtime = deadtime-(uint32_t)(127.0f*1000000000.0f/(float)HAL_RCC_GetHCLKFreq());
	  tempDT = 0b10000000 + (uint32_t)(((float)deadtime * (float)HAL_RCC_GetHCLKFreq())/(float)2000000000.0f);
	  MODIFY_REG(tmpbdtr, TIM_BDTR_DTG, tempDT);
  }
  mtr->mtimer->Instance->BDTR = tmpbdtr;
#endif

  	// Start the PWM channels, reset the counter to zero each time to avoid
	// triggering the ADC, which in turn triggers the ISR routine and wrecks the
	// startup
	mesc_init_3(_motor);
	//Set the keybits
	_motor->key_bits = UNINITIALISED_KEY + KILLSWITCH_KEY + SAFESTART_KEY;

while(_motor->MotorState == MOTOR_STATE_INITIALISING){
	//At this point, the ADCs have started and we want nothing to happen until initialisation complete
	MESCpwm_generateBreakAll();
}
	calculateGains(_motor);
	calculateVoltageGain(_motor);

#ifdef LOGGING
  _motor->logging.lognow = 1;
#endif

#ifdef USE_SPI_ENCODER
  _motor->FOC.enc_offset = ENCODER_E_OFFSET;
#endif
	//  __HAL_TIM_ENABLE_IT(_motor->stimer, TIM_IT_UPDATE);

  	  //Start the slowloop timer
  	  HAL_TIM_Base_Start(_motor->stimer);
	// Here we can auto set the prescaler to get the us input regardless of the main clock
	  __HAL_TIM_SET_PRESCALER(_motor->stimer, ((HAL_RCC_GetHCLKFreq())/ 1000000 - 1));
	  __HAL_TIM_SET_AUTORELOAD(_motor->stimer,(1000000/SLOWTIM_SCALER) / SLOW_LOOP_FREQUENCY); //Run slowloop at 100Hz
	  __HAL_TIM_ENABLE_IT(_motor->stimer, TIM_IT_UPDATE);
	  MESCinput_Init(_motor);

  	  //htim1.Instance->BDTR |=TIM_BDTR_MOE;
	  // initialising the comparators triggers the break state,
	  // so turn it back on
	  // At this point we just let the whole thing run off into interrupt land, and
	  // the fastLoop() starts to be triggered by the ADC conversion complete
	  // interrupt

  _motor->conf_is_valid = true;

  //Lock it in initialising while the offsets not completed
//	while(_motor->key_bits & UNINITIALISED_KEY){
//		_motor->MotorState = MOTOR_STATE_INITIALISING;
//		HAL_Delay(0);
//		generateBreakAll();
//	}
}

void initialiseInverter(MESC_motor_typedef *_motor){
static int Iuoff, Ivoff, Iwoff;
      Iuoff += (float)_motor->Raw.Iu;
      Ivoff += (float)_motor->Raw.Iv;
      Iwoff += (float)_motor->Raw.Iw;

      static int initcycles = 0;
      initcycles = initcycles + 1;
      //Exit the initialisation after 1000cycles
      if (initcycles == 1000) {
        calculateGains(_motor);
        calculateVoltageGain(_motor);
        _motor->FOC.flux_b = 0.001f;
        _motor->FOC.flux_a = 0.001f;

        _motor->offset.Iu =  Iuoff/initcycles;
        _motor->offset.Iv =  Ivoff/initcycles;
        _motor->offset.Iw =  Iwoff/initcycles;
        initcycles = 0;
        Iuoff = 0;
        Ivoff = 0;
        Iwoff = 0;
		if((_motor->offset.Iu>1500) &&(_motor->offset.Iu<2600)&&(_motor->offset.Iv>1500) &&(_motor->offset.Iv<2600)&&(_motor->offset.Iw>1500) &&(_motor->offset.Iw<2600)){
			//ToDo, do we want some safety checks here like offsets being roughly correct?
					_motor->MotorState = MOTOR_STATE_TRACKING;
			        _motor->key_bits &= ~UNINITIALISED_KEY;
			        _motor->mtimer->Instance->BDTR |= TIM_BDTR_MOE;
		}else{
			handleError(_motor, ERROR_STARTUP);
			//Should just loop until this succeeds
		}
      }
}



void MESC_ADC_IRQ_handler(MESC_motor_typedef *_motor){
	fastLoop(_motor);
}

// The fastloop runs at PWM timer counter top, which is when the new ADC current
// readings arrive.
// The first few clock cycles of the interrupt should not use the adc readings,
// since the currents require approximately 1us = 144 clock cycles (f405) and 72
// clock cycles (f303) to convert.
int16_t diff;
void fastLoop(MESC_motor_typedef *_motor) {
	uint32_t cycles = CPU_CYCLES;
  // Call this directly from the TIM top IRQ
  _motor->hall.current_hall_state = getHallState(); //ToDo, this macro is not applicable to dual motors
  // First thing we ever want to do is convert the ADC values
  // to real, useable numbers.
  ADCConversion(_motor);

  switch (_motor->MotorState) {

  	case MOTOR_STATE_INITIALISING:
  		initialiseInverter(_motor);
  		break;

    case MOTOR_STATE_RUN:
    	switch(_motor->MotorSensorMode){
			case MOTOR_SENSOR_MODE_SENSORLESS:
				if(_motor->options.use_hall_start){
					if(_motor->FOC.hall_start_now){
						_motor->FOC.flux_a = (1.0f-_motor->FOC.hall_IIR)*_motor->FOC.flux_a + _motor->FOC.hall_IIR*_motor->m.hall_flux[_motor->hall.current_hall_state-1][0];
						_motor->FOC.flux_b = (1.0f-_motor->FOC.hall_IIR)*_motor->FOC.flux_b + _motor->FOC.hall_IIR*_motor->m.hall_flux[_motor->hall.current_hall_state-1][1];
//						if(fabsf(_motor->FOC.Vdq.q-_motor->m.R*_motor->FOC.Idq_smoothed.q)>HALL_VOLTAGE_THRESHOLD){
							MESCfluxobs_run(_motor); //For some reason, this does not seem to work well at stationary;
							//it results in vibrations at standstill, although it smooths the transition. Therefore, start it a bit later.
//						}else{
							_motor->FOC.FOCAngle = (uint16_t)(32768.0f + 10430.0f * fast_atan2(_motor->FOC.flux_b, _motor->FOC.flux_a)) - 32768;
//						}
					}else if(_motor->FOC.enc_start_now){
						_motor->FOC.flux_a = 0.95f*_motor->FOC.flux_a + _motor->FOC.enccos * 0.05f * _motor->m.flux_linkage;
						_motor->FOC.flux_b = 0.95f*_motor->FOC.flux_b + _motor->FOC.encsin * 0.05f * _motor->m.flux_linkage;
						MESCfluxobs_run(_motor);
					}else{
						MESCfluxobs_run(_motor);
					}
				}else{
					MESCfluxobs_run(_motor);
				}
				MESCFOC(_motor);
				break;
			case MOTOR_SENSOR_MODE_HALL:
				_motor->HFI.inject = 0;
				hallAngleEstimator();
				angleObserver(_motor);
				MESCFOC(_motor);
				break;
			case MOTOR_SENSOR_MODE_OPENLOOP:
				getIncEncAngle(_motor); //Add this for setting up encoder
				OLGenerateAngle(_motor);
				MESCFOC(_motor);
				break;
			case MOTOR_SENSOR_MODE_ABSOLUTE_ENCODER:
				_motor->FOC.enc_period_count++;
				_motor->FOC.FOCAngle = _motor->FOC.enc_angle + (uint16_t)((float)(_motor->FOC.enc_period_count) * (float)_motor->FOC.enc_pwm_step);
				MESCFOC(_motor);
				break;
			case MOTOR_SENSOR_MODE_INCREMENTAL_ENCODER:
				getIncEncAngle(_motor);
				_motor->FOC.FOCAngle = _motor->FOC.enc_angle;
				MESCFOC(_motor);
				break;
    	}//End of MotorSensorMode switch
	break;

    case MOTOR_STATE_TRACKING:
#ifdef HAS_PHASE_SENSORS
		  // Track using BEMF from phase sensors
		  MESCpwm_generateBreak(_motor);
		  getRawADCVph(_motor);
		  ADCPhaseConversion(_motor);
		  MESCTrack(_motor);
		  switch(_motor->MotorSensorMode){
		  	  case MOTOR_SENSOR_MODE_HALL:
		  		  hallAngleEstimator(_motor);
		  		  angleObserver(_motor);
		  		  break;
		  	  case MOTOR_SENSOR_MODE_SENSORLESS:
				  MESCfluxobs_run(_motor);
				  if(_motor->options.use_hall_start){
					  HallFluxMonitor(_motor);
				  }
		  		  break;
		  	  case MOTOR_SENSOR_MODE_ABSOLUTE_ENCODER:
		  		  _motor->FOC.FOCAngle = _motor->FOC.enc_angle;
		  		  break;
		  	  case MOTOR_SENSOR_MODE_INCREMENTAL_ENCODER:
		  		  getIncEncAngle(_motor);
		  		  _motor->FOC.FOCAngle = _motor->FOC.enc_angle;
		  		  break;
		  	  default:
		  		  break;
		  }
#endif

      break;

    case MOTOR_STATE_OPEN_LOOP_STARTUP:
      // Same as open loop
    	_motor->FOC.openloop_step = 60;
    	OLGenerateAngle(_motor);
    	MESCFOC(_motor);
      break;

    case MOTOR_STATE_OPEN_LOOP_TRANSITION:
      // Run open loop
      // Run observer
      // RunFOC
      // Weighted average of the outputs N PWM cycles
      // Write the PWM values
      break;

    case MOTOR_STATE_IDLE:
        MESCpwm_generateBreak(_motor);
      // Do basically nothing
      break;

    case MOTOR_STATE_DETECTING:

      if ((_motor->hall.current_hall_state == 7)) { // no hall sensors detected, all GPIO pulled high
    	_motor->MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
        _motor->MotorState = MOTOR_STATE_GET_KV;
      } else if (_motor->hall.current_hall_state == 0) {
        _motor->MotorState = MOTOR_STATE_ERROR;
        MotorError = MOTOR_ERROR_HALL0;
      } else {
        // hall sensors detected
    	  _motor->MotorSensorMode = MOTOR_SENSOR_MODE_HALL;
        MESCmeasure_GetHallTable(_motor);
        MESCFOC(_motor);
      }
      break;

    case MOTOR_STATE_MEASURING:
    			// Every PWM cycle we enter this function until
                // the resistance measurement has converged at a
                // good value. Once the measurement is complete,
                // Rphase is set, and this is no longer called
          MESCmeasure_RL(_motor);
        break;

    case MOTOR_STATE_GET_KV:
      MESCmeasure_GetkV(_motor);

      break;

    case MOTOR_STATE_ERROR:
      MESCpwm_generateBreak(_motor);  // Generate a break state (software disabling all PWM)
                        // Now panic and freak out
      break;

    case MOTOR_STATE_ALIGN:
      // Turn on at a given voltage at electricalangle0;
      break;

    case MOTOR_STATE_TEST:
    	switch(TestMode){
			case TEST_TYPE_DOUBLE_PULSE:
				// Double pulse test
				MESCmeasure_DoublePulseTest(_motor);
				break;
			case TEST_TYPE_DEAD_TIME_IDENT:
				//Here we are going to pull all phases low, and then increase the duty on one phase until we register a current response.
				//This duty represents the dead time during which there is no current response
				MESCmeasure_GetDeadtime(_motor);
				break;
			case TEST_TYPE_HARDWARE_VERIFICATION:
				//Here we want a function that pulls all phases low, then all high and verifies a response
				//Then we want to show a current response with increasing phase duty
				break;


    	}
    break;

    case MOTOR_STATE_RECOVERING:
	      deadshort(_motor); //Function to startup motor from running without phase sensors
      break;

    case MOTOR_STATE_SLAMBRAKE:
      if((fabsf(_motor->Conv.Iu)>_motor->input_vars.max_request_Idq.q)||
		  (fabsf(_motor->Conv.Iv)>_motor->input_vars.max_request_Idq.q)||
		  (fabsf(_motor->Conv.Iw)>_motor->input_vars.max_request_Idq.q)){
    	  MESCpwm_generateBreak(_motor);
      }else{
    	  MESCpwm_generateEnable(_motor);
//    	  htim1.Instance->CCR1 = 0;
//    	  htim1.Instance->CCR2 = 0;
//    	  htim1.Instance->CCR3 = 0;
    	  //We use "0", since this corresponds to all high side FETs off, always, and all low side ones on, always.
    	  //This means that current measurement can continue on low side and phase shunts, so over current protection remains active.
    	  if(_motor->MotorSensorMode ==MOTOR_SENSOR_MODE_INCREMENTAL_ENCODER){
    		  getIncEncAngle(_motor);
    		  _motor->FOC.FOCAngle = _motor->FOC.enc_angle;
//     		  if((_motor->FOC.parkangle-_motor->FOC.FOCAngle)>16384){
//     			  if((_motor->FOC.parkangle-_motor->FOC.FOCAngle)>32768){
//    			  _motor->FOC.parkangle = _motor->FOC.FOCAngle+16384;
//     			  }
//    		  }
//     		  if((_motor->FOC.FOCAngle-_motor->FOC.parkangle)>16384){
//    			  if((_motor->FOC.FOCAngle-_motor->FOC.parkangle)<32767){
//    				  _motor->FOC.parkangle = _motor->FOC.FOCAngle-16384;
//    			  }
//    		  }
    		  diff =(int)(_motor->FOC.FOCAngle-_motor->FOC.parkangle);
    		  if(abs(diff)>16384){
    			  if(diff<0){
    				  _motor->FOC.parkangle = _motor->FOC.FOCAngle+16000;
    			  __NOP();
    			  }else{
    				  _motor->FOC.parkangle = _motor->FOC.FOCAngle-16000;
        			  __NOP();
    			  }
    		  }
    		  if(abs(diff)<8000){
				  _motor->FOC.Vdq.q = 0.0f;
				  _motor->FOC.Vdq.d = 0.0f;
				  _motor->FOC.park_current_now = 0.0f;
    		  }else{
    			  _motor->FOC.Idq_req.q = -_motor->FOC.park_current*(float)diff/(float)8192;//Fill with some PID logic
    			  _motor->FOC.Idq_req.d = 0.0f;//
    			  if(diff>0){
    				  _motor->FOC.Idq_req.q = _motor->FOC.Idq_req.q + _motor->FOC.park_current;
    			  }else{
    				  _motor->FOC.Idq_req.q = _motor->FOC.Idq_req.q - _motor->FOC.park_current;
    			  }
    			  MESCFOC(_motor);
    			  _motor->FOC.park_current_now = _motor->FOC.Idq_req.q;
    		  }
    	  }else{
			  _motor->FOC.Vdq.q = 0.0f;
			  _motor->FOC.Vdq.d = 0.0f;
			  _motor->FOC.park_current_now = 0.0f;
    	  }


      }
    break;
    case MOTOR_STATE_RUN_BLDC:
    	getRawADCVph(_motor);
    	ADCPhaseConversion(_motor);
    	BLDCCommute(_motor);
		__NOP();
    	break;

    default:
		_motor->MotorState = MOTOR_STATE_ERROR;
		MESCpwm_generateBreak(_motor);
		break;
  }
#ifdef SOFTWARE_ADC_REGULAR
       HAL_ADC_Start(&hadc1); //Try to eliminate the HAL call, slow and inefficient. Leaving this here for now.
        //hadc1.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
#endif

	if(_motor->options.use_lr_observer){
		  MESClrobs_Collect(_motor);
	}

#ifdef USE_SPI_ENCODER
      tle5012(_motor);
#endif

//RunPLL for all angle options
	_motor->FOC.PLL_angle = _motor->FOC.PLL_angle + (int16_t)_motor->FOC.PLL_int + (int16_t)_motor->FOC.PLL_error;
//We add the proportional error here since we did not add it last iteration
	_motor->FOC.PLL_error = _motor->FOC.PLL_kp * (int16_t)(_motor->FOC.FOCAngle - (_motor->FOC.PLL_angle & 0xFFFF));
	_motor->FOC.PLL_int = _motor->FOC.PLL_int + _motor->FOC.PLL_ki * _motor->FOC.PLL_error;
	_motor->FOC.eHz = _motor->FOC.PLL_int * _motor->FOC.pwm_frequency*0.00001526f;//1/65536

#ifdef LOGGING
	if(_motor->logging.lognow){
		static int post_error_samples;
		if(_motor->MotorState!=MOTOR_STATE_ERROR && _motor->logging.sample_now == false){
		logVars(_motor);
		post_error_samples = LOGLENGTH/2;
		}else{//If we have an error state, we want to keep the data surrounding the error log, including some sampled during and after the fault
			if(post_error_samples>1){
				logVars(_motor);
				post_error_samples--;
			}else if(post_error_samples == 1){
				_motor->logging.print_samples_now = 1;
				_motor->logging.sample_now = false;
				post_error_samples--;
			}else{
				__NOP();
			}
		}
	}
#endif
   _motor->FOC.cycles_fastloop = CPU_CYCLES - cycles;
}

// The hyperloop runs at PWM timer bottom, when the PWM is in V7 (all high)
// In this loop, we write the values of the PWM to be updated at the next update
// event (timer top) This is where we want to inject signals for measurement so
// that the next signal level takes affect right after the ADC reading In normal
// run mode, we want to increment the angle and write the next PWM values


void hyperLoop(MESC_motor_typedef *_motor) {
//Empty now, merged into fastloop with new dual interrupt routine
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


void ADCConversion(MESC_motor_typedef *_motor) {
	_motor->FOC.Idq_smoothed.d = (_motor->FOC.Idq_smoothed.d*99.0f + _motor->FOC.Idq.d)*0.01f;
	_motor->FOC.Idq_smoothed.q = (_motor->FOC.Idq_smoothed.q*99.0f + _motor->FOC.Idq.q)*0.01f;

	getRawADC(_motor);

	// Here we take the raw ADC values, offset, cast to (float) and use the
	// hardware gain values to create volt and amp variables
	//Convert the currents to real amps in SI units
	_motor->Conv.Iu = ((float)_motor->Raw.Iu - _motor->offset.Iu) * g_hw_setup.Igain;
	_motor->Conv.Iv = ((float)_motor->Raw.Iv - _motor->offset.Iv) * g_hw_setup.Igain;
	_motor->Conv.Iw = ((float)_motor->Raw.Iw - _motor->offset.Iw) * g_hw_setup.Igain;
	_motor->Conv.Vbus =	(float)_motor->Raw.Vbus * g_hw_setup.VBGain;  // Vbus

	//Check for over limit conditions. We want this after the conversion so that the
	//correct overcurrent values are logged
	//VICheck(_motor); //This uses the "raw" values, and requires an extra function call
	if (_motor->Conv.Iu > g_hw_setup.Imax){
		handleError(_motor, ERROR_OVERCURRENT_PHA);
	}
	if (_motor->Conv.Iv > g_hw_setup.Imax){
		handleError(_motor, ERROR_OVERCURRENT_PHB);
	}
	if (_motor->Conv.Iw > g_hw_setup.Imax){
		handleError(_motor,ERROR_OVERCURRENT_PHC);
	}
	if (_motor->Conv.Vbus > g_hw_setup.Vmax){
		handleError(_motor, ERROR_OVERVOLTAGE);
	}
	if (_motor->Conv.Vbus < g_hw_setup.Vmin){
		handleError(_motor, ERROR_UNDERVOLTAGE);
	}

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

#ifdef STEPPER_MOTOR //Skip the Clarke transform
    _motor->FOC.Iab.a = _motor->Conv.Iu;
    _motor->FOC.Iab.b = _motor->Conv.Iv;
#else

    // Power Variant Clark transform
    // Here we select the phases that have the lowest duty cycle to us, since
    // they should have the best current measurements
    switch(_motor->HighPhase){
		case U:
			// Clark using phase V and W
			_motor->FOC.Iab.a = -_motor->Conv.Iv -
				  _motor->Conv.Iw;
			_motor->FOC.Iab.b =
				one_on_sqrt3 * _motor->Conv.Iv -
				one_on_sqrt3 * _motor->Conv.Iw;
			break;
		case V:
			// Clark using phase U and W
			_motor->FOC.Iab.a = _motor->Conv.Iu;
			_motor->FOC.Iab.b =
				-one_on_sqrt3 * _motor->Conv.Iu -
				two_on_sqrt3 * _motor->Conv.Iw;
			break;
		case W:
			// Clark using phase U and V
			_motor->FOC.Iab.a = _motor->Conv.Iu;
			_motor->FOC.Iab.b =
				two_on_sqrt3 * _motor->Conv.Iv +
				one_on_sqrt3 * _motor->Conv.Iu;
			break;
		case N:

			if(_motor->options.use_phase_balancing){
				_motor->FOC.Iab.g = 0.33f * (_motor->Conv.Iu + _motor->Conv.Iv + _motor->Conv.Iw);
				_motor->Conv.Iu = _motor->Conv.Iu - _motor->FOC.Iab.g;
				_motor->Conv.Iv = _motor->Conv.Iv - _motor->FOC.Iab.g;
				_motor->Conv.Iw = _motor->Conv.Iw - _motor->FOC.Iab.g;
				if(fabs(_motor->FOC.Iab.g)>fabs(_motor->FOC.maxIgamma)){
					_motor->FOC.maxIgamma = _motor->FOC.Iab.g;
				}
				if(_motor->FOC.Vdq.q<2.0f){ //Reset it to reject accumulated random noise and enable multiple goes
					_motor->FOC.maxIgamma = 0.0f;
				}
			}

      // Do the full transform
	      _motor->FOC.Iab.a =
	          0.66666f * _motor->Conv.Iu -
	          0.33333f * _motor->Conv.Iv -
	          0.33333f * _motor->Conv.Iw;
	      _motor->FOC.Iab.b =
	          one_on_sqrt3 * _motor->Conv.Iv -
	          one_on_sqrt3 * _motor->Conv.Iw;
	      break;
    }//End of phase selection switch
#endif
    // Park
    _motor->FOC.Idq.d = _motor->FOC.sincosangle.cos * _motor->FOC.Iab.a +
                     _motor->FOC.sincosangle.sin * _motor->FOC.Iab.b;
    _motor->FOC.Idq.q = _motor->FOC.sincosangle.cos * _motor->FOC.Iab.b -
                     _motor->FOC.sincosangle.sin * _motor->FOC.Iab.a;
}

void ADCPhaseConversion(MESC_motor_typedef *_motor) {
	//To save clock cycles in the main run loop we only want to convert the phase voltages while tracking.
	//Convert the voltages to volts in real SI units
	_motor->Conv.Vu = (float)_motor->Raw.Vu * g_hw_setup.VBGain;
	_motor->Conv.Vv = (float)_motor->Raw.Vv * g_hw_setup.VBGain;
	_motor->Conv.Vw = (float)_motor->Raw.Vw * g_hw_setup.VBGain;
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

void hallAngleEstimator(MESC_motor_typedef *_motor) {  // Implementation using the mid point of the hall
						   	   	   	   	   	   	   	   // sensor angles, which should be much more
						   	   	   	   	   	   	   	   // reliable to generate that the edges

	if (_motor->hall.current_hall_state != _motor->hall.last_hall_state) {
		_motor->FOC.hall_update = 1;
		if (_motor->hall.current_hall_state == 0) {
			_motor->MotorState = MOTOR_STATE_ERROR;
			MotorError = MOTOR_ERROR_HALL0;
		} else if (_motor->hall.current_hall_state == 7) {
			_motor->MotorState = MOTOR_STATE_ERROR;
			MotorError = MOTOR_ERROR_HALL7;
		}
		//////////Implement the Hall table here, but the vector can be dynamically
		/// created/filled by another function/////////////
		_motor->hall.current_hall_angle = _motor->m.hall_table[_motor->hall.current_hall_state - 1][2];

		// Calculate Hall error

		uint16_t a;
		if ((a = _motor->hall.current_hall_angle - _motor->hall.last_hall_angle) < 32000) {  // Forwards
			_motor->hall.hall_error =
				  _motor->FOC.FOCAngle - _motor->m.hall_table[_motor->hall.current_hall_state - 1][0];
			_motor->hall.dir = 1.0f;
		// _motor->FOC.HallAngle = _motor->FOC.HallAngle - 5460;
		} else  {// Backwards

			_motor->hall.hall_error =
					_motor->FOC.FOCAngle - _motor->m.hall_table[_motor->hall.current_hall_state - 1][1];
			_motor->hall.dir = -1.0f;
			// _motor->FOC.HallAngle = _motor->FOC.HallAngle + 5460;
		}
		if (_motor->hall.hall_error > 32000) {
			_motor->hall.hall_error = _motor->hall.hall_error - 65536;
		}
		if (_motor->hall.hall_error < -32000) {
			_motor->hall.hall_error = _motor->hall.hall_error + 65536;
		}
	}
}

  void angleObserver(MESC_motor_typedef *_motor) {
    // This function should take the available data (hall change, BEMF crossing
    // etc...) and process it with a PLL type mechanism
    if (_motor->FOC.hall_update == 1) {
      _motor->FOC.hall_update = 0;
      _motor->hall.last_observer_period = _motor->hall.ticks_since_last_observer_change;
      float one_on_ticks = (1.0f / _motor->hall.ticks_since_last_observer_change);
      _motor->hall.one_on_last_observer_period =
          (4.0f * _motor->hall.one_on_last_observer_period + (one_on_ticks)) * 0.2f;  // ;
      _motor->hall.angle_step =
          (4.0f * _motor->hall.angle_step +
           (one_on_ticks)*_motor->m.hall_table[_motor->hall.last_hall_state - 1][3]) *
          0.2f;

      // Reset the counters, track the previous state
      _motor->hall.last_hall_state = _motor->hall.current_hall_state;
      _motor->hall.last_hall_angle = _motor->hall.current_hall_angle;
      _motor->hall.ticks_since_last_observer_change = 0;
    }

    // Run the counter
    _motor->hall.ticks_since_last_observer_change = _motor->hall.ticks_since_last_observer_change + 1;

    if (_motor->hall.ticks_since_last_observer_change <= 2.0f * _motor->hall.last_observer_period) {
      /*      _motor->FOC.FOCAngle = _motor->FOC.FOCAngle + (uint16_t)(dir*angle_step
         + one_on_last_observer_period * (-0.9f * hall_error)); //Does not
         work...
           //Why?
 */
      if (_motor->hall.dir > 0) {  // Apply a gain to the error as well as the feed forward
        // from the last hall period. Gain of 0.9-1.1 seems to work
        // well when using corrected hall positions and spacings
        _motor->FOC.FOCAngle =
            _motor->FOC.FOCAngle +
            (uint16_t)(_motor->hall.angle_step - _motor->hall.one_on_last_observer_period * _motor->hall.hall_error);
        // one_on_last_observer_period * (-0.2f * hall_error));
      } else if (_motor->hall.dir < 0.0f) {
        _motor->FOC.FOCAngle =
            _motor->FOC.FOCAngle +
            (uint16_t)(-_motor->hall.angle_step +
            		_motor->hall.one_on_last_observer_period * (-0.9f * _motor->hall.hall_error));
        // Also does not work,
        // Why??
        _motor->FOC.FOCAngle =
            _motor->FOC.FOCAngle -
            (uint16_t)(_motor->hall.angle_step +
            		_motor->hall.one_on_last_observer_period * (0.2f * _motor->hall.hall_error));
      }
    }
    if (_motor->hall.ticks_since_last_observer_change > 1500.0f) {
    	_motor->hall.ticks_since_last_observer_change = 1500.0f;
    	_motor->hall.last_observer_period = 1500.0f;  //(ticks_since_last_hall_change);
    	_motor->hall.one_on_last_observer_period =
          1.0f / _motor->hall.last_observer_period;  // / ticks_since_last_hall_change;
      _motor->FOC.FOCAngle = _motor->hall.current_hall_angle;
    }
  }

  void OLGenerateAngle(MESC_motor_typedef *_motor) {
	//_motor->FOC.PLL_int = 0.5f*_motor->FOC.openloop_step;
    _motor->FOC.FOCAngle = _motor->FOC.FOCAngle + _motor->FOC.openloop_step;
    // ToDo
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // FOC PID algorithms
  //////////////////////////////////////////////////////////////////////////////////////////


  void MESCFOC(MESC_motor_typedef *_motor) {

    // Here we are going to do a PID loop to control the dq currents, converting
    // Idq into Vdq Calculate the errors
    static MESCiq_s Idq_err;
    Idq_err.q = (_motor->FOC.Idq_req.q - _motor->FOC.Idq.q) * _motor->FOC.Iq_pgain;


    if(_motor->options.field_weakening != FIELD_WEAKENING_OFF){
		if((_motor->FOC.FW_current<_motor->FOC.Idq_req.d)&&(_motor->MotorState==MOTOR_STATE_RUN)){//Field weakenning is -ve, but there may already be d-axis from the MTPA
			Idq_err.d = (_motor->FOC.FW_current - _motor->FOC.Idq.d) * _motor->FOC.Id_pgain;
		}else{
			Idq_err.d = (_motor->FOC.Idq_req.d - _motor->FOC.Idq.d) * _motor->FOC.Id_pgain;
		}
    }else{
    	Idq_err.d = (_motor->FOC.Idq_req.d - _motor->FOC.Idq.d) * _motor->FOC.Id_pgain;
    	//if we do not use the field weakening controller, we still want to control the d axis current...
    }


    // Integral error
    _motor->FOC.Idq_int_err.d =
    		_motor->FOC.Idq_int_err.d + _motor->FOC.Id_igain * Idq_err.d * _motor->FOC.pwm_period;
    _motor->FOC.Idq_int_err.q =
    		_motor->FOC.Idq_int_err.q + _motor->FOC.Iq_igain * Idq_err.q * _motor->FOC.pwm_period;
    // Apply the integral gain at this stage to enable bounding it

    // Apply the PID
      _motor->FOC.Vdq.d = Idq_err.d + _motor->FOC.Idq_int_err.d;
      _motor->FOC.Vdq.q = Idq_err.q + _motor->FOC.Idq_int_err.q;

      // Bounding final output
      float Vmagnow2;
switch(_motor->options.sqrt_circle_lim){
case SQRT_CIRCLE_LIM_OFF:
	  //Fixed Vd and Vq limits.
    // These limits are experimental, but result in close to 100% modulation.
    // Since Vd and Vq are orthogonal, limiting Vd is not especially helpful
    // in reducing overall voltage magnitude, since the relation
    // Vout=(Vd^2+Vq^2)^0.5 results in Vd having a small effect. Vd is
    // primarily used to drive the resistive part of the field; there is no
    // BEMF pushing against Vd and so it does not scale with RPM (except for
    // cross coupling).

    // Bounding integral
	  _motor->FOC.Idq_int_err.d = clamp(_motor->FOC.Idq_int_err.d, -_motor->FOC.Vdint_max, _motor->FOC.Vdint_max);
	  _motor->FOC.Idq_int_err.q = clamp(_motor->FOC.Idq_int_err.q, -_motor->FOC.Vqint_max, _motor->FOC.Vqint_max);

    //Bounding output
    _motor->FOC.Vdq.d = clamp(_motor->FOC.Vdq.d, -_motor->FOC.Vd_max, _motor->FOC.Vd_max);
    _motor->FOC.Vdq.q = clamp(_motor->FOC.Vdq.q, -_motor->FOC.Vq_max, _motor->FOC.Vq_max);
	break;
case SQRT_CIRCLE_LIM_ON:
    Vmagnow2 = _motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q;
    //Check if the vector length is greater than the available voltage
    _motor->FOC.Voltage = sqrtf(Vmagnow2);
    if(_motor->FOC.Voltage > _motor->FOC.Vmag_max){
		  //float Vmagnow = sqrtf(Vmagnow2);
		  float one_on_Vmagnow = 1.0f/_motor->FOC.Voltage;
		  float one_on_VmagnowxVmagmax = _motor->FOC.Vmag_max*one_on_Vmagnow;
		  _motor->FOC.Vdq.d = _motor->FOC.Vdq.d*one_on_VmagnowxVmagmax;
		  _motor->FOC.Vdq.q = _motor->FOC.Vdq.q*one_on_VmagnowxVmagmax;
		  _motor->FOC.Idq_int_err.d = _motor->FOC.Idq_int_err.d*one_on_VmagnowxVmagmax;
		  _motor->FOC.Idq_int_err.q = _motor->FOC.Idq_int_err.q*one_on_VmagnowxVmagmax;

		  if(_motor->options.field_weakening == FIELD_WEAKENING_V2){
			  //Preferable to use FWV2 with the D axis circle limiter,
			  //this allows the D current to ramp all the way to max, whereas
			  //the linear sqrt circle limiter is overcome by large q axis voltage demands
	    	  //Closed loop field weakenning that works by only applying D axis current in the case where there is no duty left.
	    	  //Seems very effective at increasing speed with good stability and maintaining max torque.
			  _motor->FOC.FW_current = 0.99f*_motor->FOC.FW_current -0.01f*_motor->FOC.FW_curr_max;
	    	  //Exponentially tend towards the max FW current
		  }

    }else{
  	  if(_motor->options.field_weakening == FIELD_WEAKENING_V2){
  		  _motor->FOC.FW_current = 1.01f*_motor->FOC.FW_current + 0.0101f*_motor->FOC.FW_curr_max;
  	  }
    }

    if(_motor->options.field_weakening == FIELD_WEAKENING_V2){
  	  //Unroll the exponential ramp up, with a small extra term to ensure we do not saturate the float
  	  if(_motor->FOC.FW_current>_motor->FOC.Idq_req.d){_motor->FOC.FW_current = _motor->FOC.Idq_req.d;}
  	  if(_motor->FOC.FW_current<-_motor->FOC.FW_curr_max){_motor->FOC.FW_current = -_motor->FOC.FW_curr_max;}
    }

	break;
case SQRT_CIRCLE_LIM_VD:
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
	  Vmagnow2 = _motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q;
	  _motor->FOC.Voltage = sqrtf(Vmagnow2);
	  if(_motor->FOC.Voltage > _motor->FOC.Vmag_max){
		  _motor->FOC.Voltage = _motor->FOC.Vmag_max;
		  if(_motor->FOC.Vdq.q>0.0f){ //Positive Vq
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

	  if(_motor->options.field_weakening == FIELD_WEAKENING_V2){
	      if(_motor->FOC.Voltage > 0.95f*_motor->FOC.Vmag_max){
	    	  //Closed loop field weakenning that works by only applying D axis current in the case where there is no duty left.
	    	  //Added extra comparison statement to allow 5% excess duty which gives some headroom for the q axis PI control
	    	  //Seems very effective at increasing speed with good stability and maintaining max torque.
	    		  _motor->FOC.FW_current = 0.99f*_motor->FOC.FW_current -0.01f*_motor->FOC.FW_curr_max;
	    	  //Exponentially tend towards the max FW current
	      }else{
			  _motor->FOC.FW_current = 1.01f*_motor->FOC.FW_current + 0.0101f*_motor->FOC.FW_curr_max;
	      }//Exponentially diverge from the FW current. Note that this exponential implemented opposite to the ramp up!
	      if(_motor->FOC.FW_current>_motor->FOC.Idq_req.d){_motor->FOC.FW_current = _motor->FOC.Idq_req.d;}
	      if(_motor->FOC.FW_current<-_motor->FOC.FW_curr_max){_motor->FOC.FW_current = -_motor->FOC.FW_curr_max;}
	  }

	break;
}

	if(_motor->options.field_weakening == FIELD_WEAKENING_V1){
		  //Calculate the module of voltage applied,
		  float Vmagnow2 = _motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q; //Need to recalculate this since limitation has maybe been applied
		  //Apply a linear slope from the threshold to the max module. Similar methodology to VESC, but run in fast loop
		  //Step towards with exponential smoother
		  if(Vmagnow2>(_motor->FOC.FW_threshold*_motor->FOC.FW_threshold)){
			  _motor->FOC.FW_current = 0.95f*_motor->FOC.FW_current +
							0.05f*_motor->FOC.FW_curr_max *_motor->FOC.FW_multiplier*
							(_motor->FOC.FW_threshold - sqrtf(Vmagnow2));
		  }else{//We are outside the FW region
			  _motor->FOC.FW_current*=0.95f;//Ramp down a bit slowly
			  if(_motor->FOC.FW_current>0.1f){//We do not allow positive field weakening current, and we want it to actually go to zero eventually
				  _motor->FOC.FW_current = 0.0f;
			  }
		  }
		  //Apply the field weakening only if the additional d current is greater than the requested d current
	}

}





  void calculateFlux(MESC_motor_typedef *_motor) {
	  _motor->m.flux_linkage_max = 1.7f*_motor->m.flux_linkage;
	  _motor->m.flux_linkage_min = 0.5f*_motor->m.flux_linkage;
	  _motor->m.flux_linkage_gain = 10.0f * sqrtf(_motor->m.flux_linkage);
	  _motor->m.non_linear_centering_gain = NON_LINEAR_CENTERING_GAIN;
  }

  void calculateGains(MESC_motor_typedef *_motor) {
    _motor->FOC.pwm_period = 1.0f/_motor->FOC.pwm_frequency;
    _motor->mtimer->Instance->ARR = HAL_RCC_GetHCLKFreq()/(((float)_motor->mtimer->Instance->PSC + 1.0f) * 2*_motor->FOC.pwm_frequency);
    _motor->mtimer->Instance->CCR4 = _motor->mtimer->Instance->ARR-5; //Just short of dead center (dead center will not actually trigger the conversion)
    #ifdef SINGLE_ADC
    _motor->mtimer->Instance->CCR4 = _motor->mtimer->Instance->ARR-80; //If we only have one ADC, we need to convert early otherwise the data will not be ready in time
    #endif
    _motor->FOC.PWMmid = _motor->mtimer->Instance->ARR * 0.5f;

    _motor->FOC.ADC_duty_threshold = _motor->mtimer->Instance->ARR * 0.90f;

    calculateFlux(_motor);

    //PID controller gains
    _motor->FOC.Id_pgain = _motor->FOC.Current_bandwidth * _motor->m.L_D;
    _motor->FOC.Id_igain = _motor->m.R / _motor->m.L_D;
    // Pole zero cancellation for series PI control
    _motor->FOC.Iq_pgain = _motor->FOC.Id_pgain;
    _motor->FOC.Iq_igain = _motor->FOC.Id_igain;

	  if(_motor->FOC.FW_curr_max > 0.9f * _motor->input_vars.max_request_Idq.q){
		  _motor->FOC.FW_curr_max = 0.9f * _motor->input_vars.max_request_Idq.q; //Limit the field weakenning to 90% of the max current to avoid math errors
	  }
	_motor->m.L_QD = _motor->m.L_Q-_motor->m.L_D;
	_motor->FOC.d_polarity = 1;
  }

  void calculateVoltageGain(MESC_motor_typedef *_motor) {
    // We need a number to convert between Va Vb and raw PWM register values
    // This number should be the bus voltage divided by the ARR register
    _motor->FOC.Vab_to_PWM =
        _motor->mtimer->Instance->ARR / _motor->Conv.Vbus;
    // We also need a number to set the maximum voltage that can be effectively
    // used by the SVPWM This is equal to
    // 0.5*Vbus*MAX_MODULATION*SVPWM_MULTIPLIER*Vd_MAX_PROPORTION
    if(_motor->ControlMode != MOTOR_CONTROL_MODE_DUTY){_motor->FOC.Duty_scaler = 1.0f;}
    _motor->FOC.Vmag_max = 0.5f * _motor->Conv.Vbus *
            MAX_MODULATION * SVPWM_MULTIPLIER * _motor->FOC.Duty_scaler;
    _motor->FOC.V_3Q_mag_max =  _motor->FOC.Vmag_max * 0.75f;

    _motor->FOC.Vmag_max2 = _motor->FOC.Vmag_max*_motor->FOC.Vmag_max;
    _motor->FOC.Vd_max = 0.5f * _motor->Conv.Vbus *
                      MAX_MODULATION * SVPWM_MULTIPLIER * Vd_MAX_PROPORTION;
    _motor->FOC.Vq_max = 0.5f * _motor->Conv.Vbus *
                      MAX_MODULATION * SVPWM_MULTIPLIER * Vq_MAX_PROPORTION;

    _motor->FOC.Vdint_max = _motor->FOC.Vd_max * 0.9f; //Logic in this is to always ensure headroom for the P term
    _motor->FOC.Vqint_max = _motor->FOC.Vq_max * 0.9f;

    _motor->FOC.FW_threshold = _motor->FOC.Vmag_max * FIELD_WEAKENING_THRESHOLD;
    _motor->FOC.FW_multiplier = 1.0f/(_motor->FOC.Vmag_max*(1.0f-FIELD_WEAKENING_THRESHOLD));

    switch(_motor->HFI.Type){//When running HFI we want the bandwidth low, so we calculate it with each slow loop depending on whether we are HFIing or not
    case HFI_TYPE_NONE:
    	__NOP();
    case HFI_TYPE_45:
    	//fallthrough
    case HFI_TYPE_D:
    	//fallthrough
    case HFI_TYPE_SPECIAL:

		_motor->FOC.Id_pgain = _motor->FOC.Current_bandwidth * _motor->m.L_D;
		_motor->FOC.Id_igain = _motor->m.R / _motor->m.L_D;
		// Pole zero cancellation for series PI control
		_motor->FOC.Iq_pgain = _motor->FOC.Id_pgain;
		_motor->FOC.Iq_igain = _motor->FOC.Id_igain;
		//This is the expected current magnitude we would see based on the average inductance and the injected voltage. Not particularly reliable currently.
		//_motor->FOC.HFI_Threshold = ((HFI_VOLTAGE*sqrt2*2.0f)*_motor->FOC.pwm_period)/((_motor->m.L_D+_motor->m.L_Q)*0.5f);
		if(HFI_THRESHOLD==0.0f){
		_motor->HFI.toggle_voltage = mtr->Conv.Vbus*0.05f;
			if(_motor->HFI.toggle_voltage<3.0f){_motor->HFI.toggle_voltage = 3.0f;}
		}else{
		_motor->HFI.toggle_voltage = HFI_THRESHOLD;
		}
		break;
    }
    //////Set the fault limits
	//Set the overcurrent limit according to the requested current.
	//This is important since using the board ABS_MAX may mean the motor DC resistance is high enough that a fault never trips it.
	g_hw_setup.Imax = _motor->input_vars.max_request_Idq.q * 1.5f;
	if((g_hw_setup.Imax * 0.5f)<(0.1f * ABS_MAX_PHASE_CURRENT)){
		g_hw_setup.Imax = _motor->input_vars.max_request_Idq.q + 0.1f*ABS_MAX_PHASE_CURRENT;
	}
	if(g_hw_setup.Imax>ABS_MAX_PHASE_CURRENT){//Clamp the current limit to the board max
		g_hw_setup.Imax = ABS_MAX_PHASE_CURRENT;
	}
	//Set the over voltage limit dynamically, so that rapid spikes above the bus voltage are trapped.
	//This should be more convenient for working with PSUs and batteries interchangeably
	if(fabsf(_motor->FOC.Idq_req.q)<1.0f){
	g_hw_setup.Vmax = 	0.995f * g_hw_setup.Vmax + 0.005f * (_motor->Conv.Vbus + 0.15f * ABS_MAX_BUS_VOLTAGE);
	}
	if(g_hw_setup.Vmax>ABS_MAX_BUS_VOLTAGE)	{
		g_hw_setup.Vmax=ABS_MAX_BUS_VOLTAGE;
	}
  }

void MESC_Slow_IRQ_handler(MESC_motor_typedef *_motor){
//#ifdef SLOWLED
//	  SLOWLED->BSRR = SLOWLEDIO;
//#endif
	  slowLoop(_motor);
//#ifdef SLOWLED
//		SLOWLED->BSRR = SLOWLEDIO<<16U;
//#endif
  }
  extern uint32_t ADC_buffer[6];

float  Square(float x){ return((x)*(x));}

  void slowLoop(MESC_motor_typedef *_motor) {
// In this loop, we will fetch the throttle values, and run functions that
// are critical, but do not need to be executed very often e.g. adjustment
// for battery voltage change
///Process buttons for direction

		houseKeeping(_motor);	//General dross that keeps things ticking over, like nudging the observer
		MESCinput_Collect(_motor); //Get all the throttle inputs
		switch(_motor->options.app_type){
			case APP_NONE:
				_motor->key_bits &= ~APP_KEY;
				No_app(_motor); //No_app just sums the inputs
				break;
			case APP_VEHICLE:
				Vehicle_app(_motor);
				break;
			case APP_2:
				break;
			case APP_3:
				break;

		}


	  switch(_motor->ControlMode){
		  case MOTOR_CONTROL_MODE_TORQUE:
//Dealt with in APP_NONE
			  break;
		  case MOTOR_CONTROL_MODE_POSITION:
			  RunPosControl(_motor);
			  break;
		  case MOTOR_CONTROL_MODE_SPEED:
			  //TBC PID loop to convert eHz feedback to an iq request
			  RunSpeedControl(_motor);
			  break;
		  case MOTOR_CONTROL_MODE_DUTY:
			  _motor->FOC.Idq_prereq = _motor->input_vars.max_request_Idq;
			  //Sum the total duty request
			  float total_in = 	_motor->input_vars.ADC1_req + _motor->input_vars.ADC2_req +
					  	  	    _motor->input_vars.RCPWM_req + _motor->input_vars.UART_req + _motor->input_vars.ADC12_diff_req +
								_motor->input_vars.remote_ADC1_req + _motor->input_vars.remote_ADC2_req;
			  if(fabsf(total_in)>0.01f){
				  total_in = clamp(total_in, -1.0f, 1.0f);
				  _motor->FOC.Duty_scaler = fabsf(total_in); //Assign the duty here
			  } else {
				  total_in = 0.001f;
				  _motor->FOC.Duty_scaler = fabsf(total_in);
			  }
			  break;
		  case MOTOR_CONTROL_MODE_MEASURING:
			_motor->MotorSensorMode = MOTOR_SENSOR_MODE_OPENLOOP;
			_motor->HFI.Type = HFI_TYPE_NONE;
			_motor->FOC.Id_pgain = 0.0f;
			_motor->FOC.Iq_pgain = 0.0f;
			_motor->FOC.Id_igain = 0.0f;
			_motor->FOC.Iq_igain = 0.0f;
			_motor->FOC.openloop_step = (uint16_t)(600.0f*65536/_motor->FOC.pwm_frequency);//300Hz tone
			_motor->FOC.Idq_int_err.d = 10.0f;//1V
			_motor->FOC.Idq_int_err.q = 0.0f;//1V
			_motor->FOC.Current_bandwidth = 0.0f;
			_motor->FOC.PLL_int = 0.0f;
			_motor->FOC.PLL_ki = 0.0f;
			_motor->FOC.PLL_ki = 0.0f;
			_motor->FOC.PLL_error = 0.0f;

			_motor->m.R =10.0f*_motor->FOC.Idq_smoothed.d /(_motor->FOC.Idq_smoothed.d*_motor->FOC.Idq_smoothed.d +
					_motor->FOC.Idq_smoothed.q*_motor->FOC.Idq_smoothed.q);
			_motor->m.L_D = -10.0f*_motor->FOC.Idq_smoothed.q/(2.0f*3.1415f*600.0f*(_motor->FOC.Idq_smoothed.d*_motor->FOC.Idq_smoothed.d +
					_motor->FOC.Idq_smoothed.q*_motor->FOC.Idq_smoothed.q));
			if(_motor->MotorState !=MOTOR_STATE_ERROR){
				_motor->MotorState = MOTOR_STATE_RUN;
			}
			  break;
		  case MOTOR_CONTROL_MODE_HANDBRAKE:
			  if((_motor->MotorState==MOTOR_STATE_RUN)||(_motor->MotorState==MOTOR_STATE_TRACKING)){
				  if((fabsf(_motor->FOC.Vdq.q)<0.1f*_motor->Conv.Vbus)){//Check it is not error or spinning fast!
					  _motor->MotorState = MOTOR_STATE_SLAMBRAKE;
				  }else{//We are going fast, just disable PWM
					  _motor->MotorState = MOTOR_STATE_TRACKING;
						MESCpwm_generateBreak(_motor);
				  }
			  }
			  float req_now = (_motor->input_vars.UART_req + _motor->input_vars.max_request_Idq.q * (_motor->input_vars.ADC1_req + _motor->input_vars.ADC2_req + _motor->input_vars.RCPWM_req));

			  _motor->FOC.Idq_prereq.q = req_now;
			  if((req_now>(0.05f*_motor->input_vars.max_request_Idq.q))&&(req_now>_motor->FOC.park_current_now)&&(_motor->MotorState == MOTOR_STATE_SLAMBRAKE)){
				  _motor->MotorState = MOTOR_STATE_TRACKING;
				  _motor->ControlMode = MOTOR_CONTROL_MODE_TORQUE;
			  }
			  break;
		  default:
			  __NOP();
			  break;
	  }
	  /////////////////Handle the safe startup
	  safeStart(_motor);
	  /////////////////Handle the keybits (initialised flag, killswitch and safestart)
	  if((_motor->key_bits)){
		  _motor->FOC.Idq_prereq.q = 0.0f;
		  _motor->FOC.Idq_prereq.d = 0.0f;
	  }
		///////////////////////Run the state machine//////////////////////////////////
	switch(_motor->MotorState){
		case MOTOR_STATE_TRACKING:
			ThrottleTemperature(_motor);
			_motor->FOC.was_last_tracking = 1;
			//Seperate based on control mode. We NEED to have a fallthrough here in transition state!
			//Does not seem possible to use nested switches due to fallthrough requirement :(
			if(_motor->ControlMode == MOTOR_CONTROL_MODE_TORQUE){
				if(MESCinput_isHandbrake()){_motor->ControlMode = MOTOR_CONTROL_MODE_HANDBRAKE;}
				if(fabsf(_motor->FOC.Idq_prereq.q)>0.2f){
					#ifdef HAS_PHASE_SENSORS
					if(_motor->MotorControlType == MOTOR_CONTROL_TYPE_FOC){
						_motor->MotorState = MOTOR_STATE_RUN;
					}else if(_motor->MotorControlType == MOTOR_CONTROL_TYPE_BLDC){
						_motor->MotorState = MOTOR_STATE_RUN_BLDC;

					}
					#else
					_motor->MotorState = MOTOR_STATE_RECOVERING;
					break;
					#endif
					//fallthrough to RUN, no break!
				}else{
					//Remain in tracking
				break;
				}
			} else if(_motor->ControlMode == MOTOR_CONTROL_MODE_POSITION){
				if(_motor->MotorState!=MOTOR_STATE_ERROR){
					_motor->MotorState = MOTOR_STATE_RUN;
				}
			}else if(_motor->ControlMode == MOTOR_CONTROL_MODE_SPEED){
					if(_motor->FOC.speed_req > 10.0f){
						_motor->MotorState = MOTOR_STATE_RUN;
						//fallthrough to RUN, no break!
					} else{
					break;
					}
			}else if(_motor->ControlMode == MOTOR_CONTROL_MODE_DUTY){
				if(_motor->FOC.Duty_scaler > 0.01f){
					_motor->MotorState = MOTOR_STATE_RUN;
					//fallthrough to RUN, no break!
				} else{
				break;
				}
			}
			//end of ControlMode switch

		case MOTOR_STATE_RUN:
			calculatePower(_motor);
			ThrottleTemperature(_motor); //Gradually ramp down the Q current if motor or FETs are getting hot
			if(_motor->options.MTPA_mode){
				RunMTPA(_motor);//Process MTPA
			}

			LimitFWCurrent(_motor);//Process FW -> Iq reduction
			clampBatteryPower(_motor); //Prevent too much power being drawn from the battery

			if(_motor->options.use_lr_observer){
				MESClrobs_Run(_motor);
			}

			//Assign the Idqreq to the PI input
			_motor->FOC.Idq_req.q = _motor->FOC.Idq_prereq.q;
			_motor->FOC.Idq_req.d = _motor->FOC.Idq_prereq.d;
			if(_motor->input_vars.UART_dreq){_motor->FOC.Idq_req.d = _motor->input_vars.UART_dreq;}//Override the calcs if a specific d is requested
			MESCpwm_generateEnable(_motor);
			switch(_motor->ControlMode){
				case MOTOR_CONTROL_MODE_TORQUE:
					if(((fabsf(_motor->FOC.Idq_prereq.q)<0.1f))){//Request current small, FW not active
						if((_motor->FOC.FW_current>-0.5f)){
						_motor->MotorState = MOTOR_STATE_TRACKING;
						MESCpwm_generateBreak(_motor);
						}else{
							FWRampDown(_motor);
						}
					}
					if(MESCinput_isHandbrake()){_motor->ControlMode = MOTOR_CONTROL_MODE_HANDBRAKE;}
					break;

				case MOTOR_CONTROL_MODE_SPEED:
					if(fabsf(_motor->FOC.speed_req) < 10.0f){
						_motor->MotorState = MOTOR_STATE_TRACKING;
						MESCpwm_generateBreak(_motor);
					}
					break;
				case MOTOR_CONTROL_MODE_DUTY:
					if(_motor->FOC.Duty_scaler > 0.01f){

					}else{
						_motor->MotorState = MOTOR_STATE_TRACKING;
						MESCpwm_generateBreak(_motor);
					}
					break;
				case MOTOR_CONTROL_MODE_POSITION:
					__NOP();
				default:
					break;
			}//end of ControlMode switch

			SlowStartup(_motor);
			break;
		case MOTOR_STATE_RUN_BLDC:
			//Assign the Idqreq to the PI input
			_motor->BLDC.I_set = _motor->FOC.Idq_prereq.q;
			break;

		case MOTOR_STATE_ERROR:
				//add recovery stuff
			switch(_motor->ControlMode){
				case MOTOR_CONTROL_MODE_HANDBRAKE:
					//fallthrough
				case MOTOR_CONTROL_MODE_TORQUE:
				if(fabsf(_motor->FOC.Idq_prereq.q)<0.1f){
					_motor->MotorState = MOTOR_STATE_TRACKING;
					VICheck(_motor); //Immediately return it to error state if there is still a critical fault condition active
					clearErrors();
				}
				break;
				case MOTOR_CONTROL_MODE_SPEED:
					if(fabsf(_motor->FOC.speed_req)<0.1f){
						_motor->MotorState = MOTOR_STATE_TRACKING;
						VICheck(_motor); //Immediately return it to error state if there is still a critical fault condition active
					}
					break;
				case MOTOR_CONTROL_MODE_DUTY:
					if(fabsf(_motor->FOC.Duty_scaler)<0.01f){
						_motor->MotorState = MOTOR_STATE_TRACKING;
						VICheck(_motor); //Immediately return it to error state if there is still a critical fault condition active
					}
				default:
					break;
			}
			break;
		case MOTOR_STATE_SLAMBRAKE:
			__NOP();
			//We might want to do something if there is a handbrake state? Like exiting this state?
			break;
		default:
			__NOP();
			//This accounts for all the initialising, test, measuring... procedures.
			//We basically just want to do nothing and let them get on with their job.
			break;
	}
	/////////////////End of Switch state machine///////////////////////////////

	calculateVoltageGain(_motor);
}


void MESCTrack(MESC_motor_typedef *_motor) {
    // here we are going to do the clark and park transform of the voltages to
    // get the VaVb and VdVq These can be handed later to the observers and used
    // to set the integral terms

	//Accumulate the current offsets while there is no current (tri-stated)
	_motor->offset.Iu = 0.9999f*_motor->offset.Iu +0.0001f*(float)_motor->Raw.Iu;
	_motor->offset.Iv = 0.9999f*_motor->offset.Iv +0.0001f*(float)_motor->Raw.Iv;
	_motor->offset.Iw = 0.9999f*_motor->offset.Iw +0.0001f*(float)_motor->Raw.Iw;


	// Clark transform
	_motor->FOC.Vab.a =
		0.666f * (_motor->Conv.Vu -
				  0.5f * ((_motor->Conv.Vv) +
						  (_motor->Conv.Vw)));
	_motor->FOC.Vab.b =
		0.666f *
		(sqrt3_on_2 * ((_motor->Conv.Vv) -
					   (_motor->Conv.Vw)));

	sin_cos_fast(_motor->FOC.FOCAngle, &_motor->FOC.sincosangle.sin, &_motor->FOC.sincosangle.cos);

	// Park transform

	_motor->FOC.Vdq.d = _motor->FOC.sincosangle.cos * _motor->FOC.Vab.a +
					  _motor->FOC.sincosangle.sin * _motor->FOC.Vab.b;
	_motor->FOC.Vdq.q = _motor->FOC.sincosangle.cos * _motor->FOC.Vab.b -
					  _motor->FOC.sincosangle.sin * _motor->FOC.Vab.a;
	_motor->FOC.Idq_int_err.q = _motor->FOC.Vdq.q;
	_motor->FOC.Idq_int_err.d = _motor->FOC.Vdq.d;

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

	  		if(countdown == 1||(((_motor->FOC.Iab.a*_motor->FOC.Iab.a+_motor->FOC.Iab.b*_motor->FOC.Iab.b)>DEADSHORT_CURRENT*DEADSHORT_CURRENT)&&countdown<9))
	  				{
	  					//Need to collect the ADC currents here
	  					MESCpwm_generateBreak(_motor);
	  					//Calculate the voltages in the alpha beta phase...
	  					IacalcDS = _motor->FOC.Iab.a;
	  					IbcalcDS = _motor->FOC.Iab.b;
	  					VacalcDS = -_motor->m.L_D*_motor->FOC.Iab.a/((9.0f-(float)countdown)*_motor->FOC.pwm_period);
	  					VbcalcDS = -_motor->m.L_D*_motor->FOC.Iab.b/((9.0f-(float)countdown)*_motor->FOC.pwm_period);
	  					//Calculate the phase angle
	  					//TEST LINE angleDS = (uint16_t)(32768.0f + 10430.0f * fast_atan2(VbcalcDS, VacalcDS)) - 32768;// +16384;

	  					 angleDS = (uint16_t)(32768.0f + 10430.0f * fast_atan2(VbcalcDS, VacalcDS)) - 32768 -16384;
	  					//Shifting by 1/4 erev does not work for going backwards. Need to rethink.
	  					//Problem is, depending on motor direction, the sign of the voltage generated swaps for the same rotor position.
	  					//The atan2(flux linkages) is stable under this regime, but the same for voltage is not.
	  					_motor->FOC.FOCAngle = angleDS;//
	  					sin_cos_fast(_motor->FOC.FOCAngle, &_motor->FOC.sincosangle.sin, &_motor->FOC.sincosangle.cos);

	  					//Park transform it to get VdVq
	  					VdcalcDS = _motor->FOC.sincosangle.cos * VacalcDS +
	  				                      _motor->FOC.sincosangle.sin * VbcalcDS;
	  					VqcalcDS = _motor->FOC.sincosangle.cos * VbcalcDS -
	  				                      _motor->FOC.sincosangle.sin * VacalcDS;
	  					//Preloading the observer
	  					FLaDS = _motor->FOC.flux_observed*_motor->FOC.sincosangle.cos;
	  					FLbDS = _motor->FOC.flux_observed*_motor->FOC.sincosangle.sin;
	  		//Angle Errors for debugging
	  					angleErrorDSENC = angleDS-_motor->FOC.enc_angle;
	  		//Do actual preloading
	  					_motor->FOC.flux_a = FLaDS;
	  					_motor->FOC.flux_b = FLbDS;
	  					_motor->FOC.Ia_last = 0.0f;
	  					_motor->FOC.Ib_last = 0.0f;
	  					_motor->FOC.Idq_int_err.d = VdcalcDS;
	  					_motor->FOC.Idq_int_err.q = VqcalcDS;
	  		//Next PWM cycle it  will jump to running state,
	  					MESCFOC(_motor);
	  					countdown_cycles = 9-countdown;
	  					countdown = 1;
	  		}
	  		if(countdown > 10){
	  			MESCpwm_generateBreak(_motor);
	  			_motor->mtimer->Instance->CCR1 = 50;
	  			_motor->mtimer->Instance->CCR2 = 50;
	  			_motor->mtimer->Instance->CCR3 = 50;
	  			//Preload the timer at mid
	  		}
	  		if(countdown <= 10 && countdown>1 ){
	  			_motor->mtimer->Instance->CCR1 = 50;
	  			_motor->mtimer->Instance->CCR2 = 50;
	  			_motor->mtimer->Instance->CCR3 = 50;
	  			MESCpwm_generateEnable(_motor);
	  		}
	  		if(countdown == 1 ){
					countdown = 15; //We need at least a few cycles for the current to relax
									//to zero in case of rapid switching between states
  					_motor->MotorState = MOTOR_STATE_RUN;

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

  void tle5012(MESC_motor_typedef *_motor)
  {
#ifdef USE_SPI_ENCODER
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




void HallFluxMonitor(MESC_motor_typedef *_motor){
	if(fabsf(_motor->FOC.Vdq.q)>MIN_HALL_FLUX_VOLTS){ //Are we actually spinning at a reasonable pace?
		if((_motor->hall.current_hall_state>0)&&(_motor->hall.current_hall_state<7)){
			_motor->m.hall_flux[_motor->hall.current_hall_state - 1][0] = 	0.999f*_motor->m.hall_flux[_motor->hall.current_hall_state - 1][0] +
																			0.001f*_motor->FOC.flux_a;
			//take a slow average of the alpha flux linked and store it for later preloading
			//the observer during very low speed conditions. There is a slight bias towards
			//later values of flux linked, which is probably good.
			_motor->m.hall_flux[_motor->hall.current_hall_state - 1][1] = 	0.999f*_motor->m.hall_flux[_motor->hall.current_hall_state - 1][1] +
																			0.001f*_motor->FOC.flux_b;
		}
		_motor->FOC.hall_initialised = 1;
	}
}
void getIncEncAngle(MESC_motor_typedef *_motor){
	if(_motor->FOC.encoder_polarity_invert){
		_motor->FOC.enc_angle = _motor->m.pole_pairs*(65536-(_motor->FOC.enc_ratio*(uint16_t)_motor->enctimer->Instance->CNT-_motor->FOC.enc_ratio*(uint16_t)_motor->enctimer->Instance->CCR3)) + _motor->FOC.enc_offset;

	}else{
		_motor->FOC.enc_angle = _motor->m.pole_pairs*((_motor->FOC.enc_ratio*(uint16_t)_motor->enctimer->Instance->CNT-_motor->FOC.enc_ratio*(uint16_t)_motor->enctimer->Instance->CCR3)) + _motor->FOC.enc_offset;
	}
}

void  logVars(MESC_motor_typedef *_motor){

	_motor->logging.Vbus[_motor->logging.current_sample] = _motor->Conv.Vbus;
	_motor->logging.Iu[_motor->logging.current_sample] = _motor->Conv.Iu;
	_motor->logging.Iv[_motor->logging.current_sample] = _motor->Conv.Iv;
	_motor->logging.Iw[_motor->logging.current_sample] = _motor->Conv.Iw;
	_motor->logging.Vd[_motor->logging.current_sample] = _motor->FOC.Vdq.d;
	_motor->logging.Vq[_motor->logging.current_sample] = _motor->FOC.Vdq.q;
	_motor->logging.angle[_motor->logging.current_sample] = _motor->FOC.FOCAngle;
	_motor->logging.current_sample++;
	if(_motor->logging.current_sample>=LOGLENGTH){
		_motor->logging.current_sample = 0;
	}
}






void SlowStartup(MESC_motor_typedef *_motor){
	switch(_motor->SLStartupSensor){
	case STARTUP_SENSOR_HALL:
		if((fabsf(_motor->FOC.Vdq.q-_motor->m.R*_motor->FOC.Idq_smoothed.q)<_motor->FOC.hall_transition_V)&&(_motor->FOC.hall_initialised)&&(_motor->hall.current_hall_state>0)&&(_motor->hall.current_hall_state<7)){
				_motor->FOC.hall_start_now = 1;
		}else if((fabsf(_motor->FOC.Vdq.q-_motor->m.R*_motor->FOC.Idq_smoothed.q)>_motor->FOC.hall_transition_V+2.0f)||(_motor->hall.current_hall_state<1)||(_motor->hall.current_hall_state>6)){
			_motor->FOC.hall_start_now = 0;
		}
		break;
	case STARTUP_SENSOR_PWM_ENCODER:
		if((fabsf(_motor->FOC.Vdq.q-_motor->m.R*_motor->FOC.Idq_smoothed.q)<_motor->FOC.hall_transition_V)&&(_motor->FOC.encoder_OK)){
				_motor->FOC.enc_start_now = 1;
		}else if((fabsf(_motor->FOC.Vdq.q-_motor->m.R*_motor->FOC.Idq_smoothed.q)>_motor->FOC.hall_transition_V+2.0f)||!(_motor->FOC.encoder_OK)){
			_motor->FOC.enc_start_now = 0;
		}
		break;
	case STARTUP_SENSOR_HFI:
		MESChfi_Slow(_motor);
		break;
	default: //We are not using a startup mechanism
		_motor->FOC.hall_start_now = 0;
		_motor->FOC.enc_start_now = 0;
		_motor->HFI.inject = 0;
	}

}


void RunMTPA(MESC_motor_typedef *_motor){
	//Run MTPA (Field weakening seems to have to go in  the fast loop to be stable)
	float i_mag = 0;
	if(_motor->m.L_QD>0.0f){
		switch(_motor->options.MTPA_mode){
		case MTPA_NONE:
			//Nothing
			break;
		case MTPA_REQ:
			//MTPA equation
			i_mag = _motor->FOC.Idq_prereq.q;
//			_motor->FOC.id_mtpa = _motor->m.flux_linkage/(4.0f*_motor->m.L_QD) - sqrtf((_motor->m.flux_linkage*_motor->m.flux_linkage/(16.0f*_motor->m.L_QD*_motor->m.L_QD))+_motor->FOC.Idq_prereq.q*_motor->FOC.Idq_prereq.q*0.5f);
			break;
		case MTPA_MAG:
			//Calculate magnitude of currents
			i_mag = sqrtf(_motor->FOC.Idq_smoothed.q * _motor->FOC.Idq_smoothed.q+_motor->FOC.Idq_smoothed.d * _motor->FOC.Idq_smoothed.d);
			break;

		case MTPA_Q:
			i_mag = _motor->FOC.Idq_smoothed.q;
			break;
		}//End of switch

		//MTPA equation
		_motor->FOC.id_mtpa = _motor->m.flux_linkage/(4.0f*_motor->m.L_QD) - sqrtf((_motor->m.flux_linkage*_motor->m.flux_linkage/(16.0f*_motor->m.L_QD*_motor->m.L_QD)) + (i_mag * i_mag) * 0.5f);
		//Residual to Iq
		if(fabsf(_motor->FOC.Idq_prereq.q)>fabsf(_motor->FOC.id_mtpa)){
			_motor->FOC.iq_mtpa = sqrtf(_motor->FOC.Idq_prereq.q * _motor->FOC.Idq_prereq.q - _motor->FOC.id_mtpa * _motor->FOC.id_mtpa);
		}
		else{
			_motor->FOC.iq_mtpa = 0.0f;
		}
		//Set Id_prereq
		_motor->FOC.Idq_prereq.d = _motor->FOC.id_mtpa;
		//Set Iq_prereq
		if(_motor->FOC.Idq_prereq.q>0.0f){
			_motor->FOC.Idq_prereq.q = _motor->FOC.iq_mtpa;
		}
		else{
			_motor->FOC.Idq_prereq.q = -_motor->FOC.iq_mtpa;
		}
	}
}

void calculatePower(MESC_motor_typedef *_motor){
////// Calculate the current power
		_motor->FOC.currentPower.d = 1.5f*(_motor->FOC.Vdq.d*_motor->FOC.Idq_smoothed.d);
		_motor->FOC.currentPower.q = 1.5f*(_motor->FOC.Vdq.q*_motor->FOC.Idq_smoothed.q);
		_motor->FOC.Ibus = (_motor->FOC.currentPower.d + _motor->FOC.currentPower.q) /_motor->Conv.Vbus;
}

void LimitFWCurrent(MESC_motor_typedef *_motor){
    //Account for Field weakening current
    //MTPA is already conservative of the current limits
    float mag = (Square(_motor->FOC.Idq_prereq.q) + Square(_motor->FOC.FW_current));
    if(mag>Square(_motor->input_vars.max_request_Idq.q)){
    	float Iqmax2 = Square(_motor->input_vars.max_request_Idq.q)-Square(_motor->FOC.FW_current);
    	if(Iqmax2>0){//Avoid hardfault
			if(_motor->FOC.Idq_prereq.q>0){
				_motor->FOC.Idq_prereq.q = sqrtf(Iqmax2);
			}else{
				_motor->FOC.Idq_prereq.q = -sqrtf(Iqmax2);
			}
    	}else{//Negative result, FW larger than allowable current
    		_motor->MotorState = MOTOR_STATE_ERROR;
    		handleError(_motor, ERROR_MATH);
    		_motor->FOC.FW_current = 0.0f;
    	}
    }

}

void clampBatteryPower(MESC_motor_typedef *_motor){
/////// Clamp the max power taken from the battery
    _motor->FOC.reqPower = 1.5f*fabsf(_motor->FOC.Vdq.q * _motor->FOC.Idq_prereq.q);
    if (_motor->FOC.reqPower > _motor->m.Pmax) {
    	if(_motor->FOC.Idq_prereq.q > 0.0f){
    		_motor->FOC.Idq_prereq.q = _motor->m.Pmax / (fabsf(_motor->FOC.Vdq.q)*1.5f);
    	}else{
    		_motor->FOC.Idq_prereq.q = -_motor->m.Pmax / (fabsf(_motor->FOC.Vdq.q)*1.5f);
    	}
    }
}
void houseKeeping(MESC_motor_typedef *_motor){
	////// Unpuc the observer kludge
	// The observer gets into a bit of a state if it gets close to
	// flux linked = 0 for both accumulators, the angle rapidly changes
	// as it oscillates around zero. Solution... just kludge it back out.
	// This only happens at stationary when it is useless anyway.
	if ((_motor->FOC.flux_a * _motor->FOC.flux_a + _motor->FOC.flux_b * _motor->FOC.flux_b) <
		0.25f * _motor->FOC.flux_observed * _motor->FOC.flux_observed) {
		_motor->FOC.flux_a = 2.5f * _motor->FOC.flux_a;//_motor->FOC.flux_observed;
		_motor->FOC.flux_b = 2.5f * _motor->FOC.flux_b;//_motor->FOC.flux_observed;
		//This was altered because otherwise basing the flux on the observed flux
		//causes issues a step change in direction, so at low speed - e.g. during hall sensor startup - it causes instability.
	}

	//Speed tracker
	if(abs(_motor->FOC.PLL_int)>10000.0f){
		//The PLL has run away locking on to aliases; 10000 implies 6.5 pwm periods per sin wave, which is ~3000eHz, 180kerpm at 20kHz PWM frequency.
		//While it IS possible to run faster than this, it is not a sensible use case and will not be supported.
		_motor->FOC.PLL_int = 0;
	}
	//Translate the eHz to eRPM
	if(_motor->m.pole_pairs>0){//avoid divide by zero
	_motor->FOC.mechRPM = _motor->FOC.eHz*60.0f/(float)(_motor->m.pole_pairs);
	}
	//Shut down if we are burning the hall sensors //Legacy code, can probably be removed...
//	if(getHallState()==0){//This happens when the hall sensors overheat it seems.
//	  	  if (MotorError == MOTOR_ERROR_NONE) {
//	  		    speed_motor_limiter();
//	  	  }
//	  	  MotorError = MOTOR_ERROR_HALL0;
//	    }else /*if(getHallState()==7){
//	  	  MotorError = MOTOR_ERROR_HALL7;
//	    } else */{
//	  	  if (MotorError != MOTOR_ERROR_NONE) {
//	  		  // TODO speed_road();
//	  	  }
//	  	  MotorError = MOTOR_ERROR_NONE;
//	    }
}
void FWRampDown(MESC_motor_typedef *_motor){
	//Ramp down the field weakening current
	//Do NOT assign motorState here, since it could override error states
	if(_motor->FOC.Vdq.q <0.0f){
		_motor->FOC.Idq_req.q = 0.2f; //Apply a brake current
	}
	if(_motor->FOC.Vdq.q >0.0f){
		_motor->FOC.Idq_req.q = -0.2f; //Apply a brake current
	}
}

static void handleThrottleTemperature(MESC_motor_typedef *_motor, float const T, float * const dTmax, int const errorcode )
{
	float dT = 0.0f;
	TEMPState const temp_state = temp_check( &_motor->Raw.Motor_temp, T, &dT );
#define TMAX(a,b) (((a) > (b)) ? (a) : (b))
	*dTmax = TMAX( *dTmax, dT );
#undef TMAX
	if (temp_state == TEMP_STATE_OVERHEATED)
	{
	   handleError( _motor, errorcode );
	}
}

float dTmax = 0.0f;
void ThrottleTemperature(MESC_motor_typedef *_motor){
	dTmax = 0.0f;

	_motor->Conv.MOSu_T  = 0.99f *_motor->Conv.MOSu_T  + 0.01f * temp_read( &_motor->Raw.MOS_temp  , _motor->Raw.MOSu_T  );
	_motor->Conv.MOSv_T  = 0.99f *_motor->Conv.MOSv_T  + 0.01f * temp_read( &_motor->Raw.MOS_temp  , _motor->Raw.MOSv_T  );
	_motor->Conv.MOSw_T  = 0.99f *_motor->Conv.MOSw_T  + 0.01f * temp_read( &_motor->Raw.MOS_temp  , _motor->Raw.MOSw_T  );
	_motor->Conv.Motor_T = 0.99f *_motor->Conv.Motor_T + 0.01f * temp_read( &_motor->Raw.Motor_temp, _motor->Raw.Motor_T );

	handleThrottleTemperature( _motor, _motor->Conv.MOSu_T , &dTmax, ERROR_OVERTEMPU );
	handleThrottleTemperature( _motor, _motor->Conv.MOSv_T , &dTmax, ERROR_OVERTEMPV );
	handleThrottleTemperature( _motor, _motor->Conv.MOSw_T , &dTmax, ERROR_OVERTEMPW );
	if(_motor->options.has_motor_temp_sensor){
		handleThrottleTemperature( _motor, _motor->Conv.Motor_T, &dTmax, ERROR_OVERTEMP_MOTOR );
	}

	_motor->FOC.T_rollback = (1.0f-dTmax/(_motor->Raw.MOS_temp.limit.Tmax-_motor->Raw.MOS_temp.limit.Thot));
	if(_motor->FOC.T_rollback<=0.0f){
		_motor->FOC.T_rollback = 0.0f;
	}
	if(_motor->FOC.T_rollback>1.0f){
		_motor->FOC.T_rollback = 1.0f;
	}
	if(_motor->FOC.Idq_prereq.q>(_motor->FOC.T_rollback * _motor->input_vars.max_request_Idq.q)){_motor->FOC.Idq_prereq.q = _motor->FOC.T_rollback * _motor->input_vars.max_request_Idq.q;}
	if(_motor->FOC.Idq_prereq.q<(_motor->FOC.T_rollback * _motor->input_vars.min_request_Idq.q)){_motor->FOC.Idq_prereq.q = _motor->FOC.T_rollback * _motor->input_vars.min_request_Idq.q;}
}

void safeStart(MESC_motor_typedef *_motor){
	if((_motor->FOC.Idq_req.q == 0.0f)&&(_motor->FOC.Idq_prereq.q == 0.0f)){
		_motor->safe_start[1]++;
	}else if(_motor->safe_start[1]<_motor->safe_start[0] ){
		_motor->safe_start[1]=0;
	}
	if(_motor->safe_start[1] >=_motor->safe_start[0]){
		_motor->safe_start[1] = _motor->safe_start[0];
		_motor->key_bits &= ~SAFESTART_KEY;
	}
}



//Speed controller
void RunSpeedControl(MESC_motor_typedef *_motor){
	float speed_error;
	  if(_motor->MotorState == MOTOR_STATE_RUN){

		speed_error = _motor->FOC.speed_kp*(_motor->FOC.speed_req - _motor->FOC.eHz);
		//Bound the proportional term before we do anything with it
		//We use the symetric terms here to allow fast PID ramps
		speed_error = clamp(speed_error, -_motor->input_vars.max_request_Idq.q, _motor->input_vars.max_request_Idq.q);

		_motor->FOC.speed_error_int = _motor->FOC.speed_error_int + speed_error * _motor->FOC.speed_ki;
		//Bound the integral term...
		//Again, using symmetric terms
		_motor->FOC.speed_error_int = clamp(_motor->FOC.speed_error_int, -_motor->input_vars.max_request_Idq.q, _motor->input_vars.max_request_Idq.q);

		//Apply the PID
		_motor->FOC.Idq_prereq.q = _motor->FOC.speed_error_int + speed_error;
		//Bound the overall...
		//Now we use asymmetric terms to stop it regenerating too much
		_motor->FOC.Idq_prereq.q = clamp(_motor->FOC.Idq_prereq.q, _motor->input_vars.min_request_Idq.q, _motor->input_vars.max_request_Idq.q);

	  } else {
		  //Set zero
		  _motor->FOC.Idq_prereq.q = 0.0f;
		  _motor->FOC.speed_error_int = 0.0f;
	  }
}

void MESC_IC_Init(
#ifdef IC_TIMER
TIM_HandleTypeDef _IC_TIMER
#endif
){
#ifdef IC_TIMER
	_IC_TIMER.Instance-> SMCR = 84;
	  _IC_TIMER.Instance-> DIER = 3;
	  _IC_TIMER.Instance-> SR = 0;
	  _IC_TIMER.Instance-> CCMR1 = 513;
	  _IC_TIMER.Instance-> CCER = 49;
	  _IC_TIMER.Instance-> ARR = 65000;
	  _IC_TIMER.Instance-> DMAR = 1;
#ifdef IC_TIMER_RCPWM
	  _IC_TIMER.Instance->PSC = (HAL_RCC_GetHCLKFreq()/(1000000*SLOWTIM_SCALER))-1;
#else //RCtimer is used for PWM encoder
	  _IC_TIMER.Instance->PSC = (HAL_RCC_GetHCLKFreq()/(4119000*SLOWTIM_SCALER))-1;
//The encoder PWM timers have a nominal frequency of 1kHz with 4119 levels

#endif
	  IC_TIM_GPIO->MODER |= MODE_AF<<(2*IC_TIM_IONO);
	  IC_TIM_GPIO->AFR[0] |=0x2<<(IC_TIM_IONO*4);
	  //__HAL_TIM_ENABLE_IT(_IC_TIMER,TIM_IT_UPDATE);
	  _IC_TIMER.Instance-> CR1 = 5;
#endif
}

uint32_t SRtemp2, SRtemp3;
void MESC_IC_IRQ_Handler(MESC_motor_typedef *_motor, uint32_t SR, uint32_t CCR1, uint32_t CCR2){
#ifdef IC_TIMER_RCPWM
	if((SR & 0x4)&&!(SR&0x1)){
		SRtemp2 = SR;
		_motor->input_vars.pulse_recieved = 1;
		_motor->input_vars.IC_duration = CCR1;
		_motor->input_vars.IC_pulse = CCR2;
	}if(SR & 0x1){
		SRtemp3 = SR;
		_motor->input_vars.pulse_recieved = 0;
	}
#endif
#ifdef IC_TIMER_ENCODER //This will be for the encoder I guess...
	//The encoder PWM timers have a nominal frequency of 1kHz with 4119 levels
	if((SR & 0x2)&&!(SR&0x1)){
		SRtemp2 = SR;
		_motor->FOC.encoder_duration = CCR1;
		_motor->FOC.encoder_pulse = CCR2;
		_motor->FOC.encoder_OK = 1;

		if(CCR2<14||CCR1<3500||CCR1>4500){
			//Handle the error?
		}
		if(CCR2<16){//No error but need to stop it from underflowing the following math
			CCR2 = 16;
		}
		uint16_t temp_enc_ang;
		temp_enc_ang = _motor->FOC.enc_offset +
						(uint16_t)(((65536*(CCR2-16))/(CCR1-24)*(uint32_t)_motor->m.pole_pairs)%65536);
//Set the angles used and zero the counter
		if(_motor->FOC.encoder_polarity_invert){
			_motor->FOC.last_enc_period = _motor->FOC.enc_period_count;
			_motor->FOC.enc_period_count = 0;
			_motor->FOC.enc_angle = 65536 - temp_enc_ang;
		} else{
			_motor->FOC.last_enc_period = _motor->FOC.enc_period_count;
			_motor->FOC.enc_angle = temp_enc_ang;
			_motor->FOC.enc_period_count = 0;
		}

	//Calculate the deltas and steps
		_motor->FOC.enc_pwm_step = 0.8f*_motor->FOC.enc_pwm_step +
				0.2f*(((int16_t)(_motor->FOC.enc_angle - _motor->FOC.last_enc_angle))/(_motor->FOC.last_enc_period + 0.1f));
		_motor->FOC.last_enc_angle = _motor->FOC.enc_angle;
	}
	//For sensorless-PWM encoder combined mode
	//Calculate the sin and cos coefficients for future use in the flux observer
	sin_cos_fast((_motor->FOC.enc_angle), &_motor->FOC.encsin, &_motor->FOC.enccos);

	if(SR & 0x1||_motor->FOC.encoder_pulse<14||_motor->FOC.encoder_pulse>(_motor->FOC.encoder_duration-7)){
		SRtemp3 = SR;
		_motor->FOC.encoder_OK = 0;
	}
#endif
}




  // clang-format on
