/*
 **
 ******************************************************************************
 * @file           : MESCfoc.c
 * @brief          : FOC running code and ADC buffers
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

#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1, hadc2, hadc3, hadc4;

float one_on_sqrt6 = 0.408248;
float one_on_sqrt3 = 0.577350;
float one_on_sqrt2 = 0.707107;
float sqrt_two_on_3 = 0.816497;
float sqrt3_2 = 1.22474;
float sqrt2 = 1.41421;
float sqrt1_2 = 0.707107;
float sqrt3_on_2 = 0.866025;
float two_on_sqrt3 = 1.73205;
int adc_conv_end;
static float flux_linked_alpha = 0.00501f;
static float flux_linked_beta = 0.00511f;
static float Lqd_diff;

MESCfoc_s foc_vars;
MESCtest_s test_vals;
foc_measurement_t measurement_buffers;  // fixme: floating function prototype
input_vars_t input_vars;

void MESCInit() {

  mesc_init_1();

  HAL_Delay(3000);  // Give the everything else time to start up (e.g. throttle,
                    // controller, PWM source...)

  mesc_init_2();

  hw_init();  // Populate the resistances, gains etc of the PCB - edit within
              // this function if compiling for other PCBs
  // motor.Rphase = 0.1; //Hack to make it skip over currently not used motor
  // parameter detection
  foc_vars.initing = 1;  // Tell it we ARE initing...

  foc_vars.hall_forwards_adjust = 5460;
  foc_vars.hall_backwards_adjust = 5460;

  flux_linked_alpha = 0.00501f;
  flux_linked_beta = -0.00401f;

  measurement_buffers.ADCOffset[0] = 0;
  measurement_buffers.ADCOffset[1] = 0;
  measurement_buffers.ADCOffset[2] = 0;

  // Start the PWM channels, reset the counter to zero each time to avoid
  // triggering the ADC, which in turn triggers the ISR routine and wrecks the
  // startup
  mesc_init_3();
#ifdef USE_ENCODER
  foc_vars.enc_offset = ENCODER_E_OFFSET;
#endif

  InputInit();

  //htim1.Instance->BDTR |=TIM_BDTR_MOE;
	  // initialising the comparators triggers the break state,
	  // so turn it back on
	  // At this point we just let the whole thing run off into interrupt land, and
	  // the fastLoop() starts to be triggered by the ADC conversion complete
	  // interrupt

}

void InputInit(){

	input_vars.max_request_Idq[0] = 0.0f; //Not supporting d-axis input current for now
	input_vars.min_request_Idq[0] = 0.0f;
	input_vars.max_request_Idq[1] = MAX_IQ_REQUEST;
	input_vars.min_request_Idq[1] = -MAX_IQ_REQUEST;

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

	input_vars.adc1_gain[0] = (input_vars.max_request_Idq[0])/(input_vars.adc1_MAX-input_vars.adc1_MIN);
	input_vars.adc1_gain[1] = (input_vars.max_request_Idq[1])/(input_vars.adc1_MAX-input_vars.adc1_MIN);

	input_vars.adc2_gain[0] = (input_vars.max_request_Idq[0])/(input_vars.adc2_MAX-input_vars.adc2_MIN);
	input_vars.adc2_gain[1] = (input_vars.max_request_Idq[1])/(input_vars.adc2_MAX-input_vars.adc2_MIN);

	//RCPWM forward gain//index [0][x] is used for Idq requests for now, might support asymmetric brake and throttle later
	input_vars.RCPWM_gain[0][0] = (input_vars.max_request_Idq[0])/((float)input_vars.IC_pulse_MAX - (float)input_vars.IC_pulse_MID - (float)input_vars.IC_pulse_DEADZONE);
	input_vars.RCPWM_gain[0][1] = (input_vars.max_request_Idq[1])/(((float)input_vars.IC_pulse_MID - (float)input_vars.IC_pulse_DEADZONE)-(float)input_vars.IC_pulse_MIN);

	input_vars.input_options = DEFAULT_INPUT;
	input_vars.ADC1_polarity = ADC1_POLARITY;
	input_vars.ADC2_polarity = ADC2_POLARITY;

	input_vars.Idq_req_UART[0] =0;
	input_vars.Idq_req_RCPWM[0] =0;
	input_vars.Idq_req_ADC1[0] =0;
	input_vars.Idq_req_ADC2[0] =0;
	input_vars.Idq_req_UART[1] =0;
	input_vars.Idq_req_RCPWM[1] =0;
	input_vars.Idq_req_ADC1[1] =0;
	input_vars.Idq_req_ADC2[1] =0;

}

// This should be the only function needed to be added into the PWM interrupt
// for MESC to run Ensure that it is followed by the clear timer update
// interrupt
void MESC_PWM_IRQ_handler() {
  if (htim1.Instance->CNT > 512) {
    foc_vars.IRQentry = debugtim.Instance->CNT;
    fastLoop();
    foc_vars.IRQexit = debugtim.Instance->CNT - foc_vars.IRQentry;
    foc_vars.FLrun++;
  } else {
    foc_vars.IRQentry = debugtim.Instance->CNT;
    hyperLoop();
    foc_vars.IRQexit = debugtim.Instance->CNT - foc_vars.IRQentry;
    foc_vars.VFLrun++;
  }
}

// The fastloop runs at PWM timer counter top, which is when the new ADC current
// readings arrive.
// The first few clock cycles of the interrupt should not use the adc readings,
// since the currents require approximately 1us = 144 clock cycles (f405) and 72
// clock cycles (f303) to convert.

static int current_hall_state;

void fastLoop() {
  // Call this directly from the TIM top IRQ
  current_hall_state = getHallState();
  // First thing we ever want to do is convert the ADC values
  // to real, useable numbers.
  ADCConversion();
  adc_conv_end = htim1.Instance->CNT;  // track the ADC conversion time

  switch (MotorState) {
    case MOTOR_STATE_RUN:
      // transform
      //      if (MotorControlType ==
      //          MOTOR_CONTROL_TYPE_BLDC) {
    	// BLDC is hopefully just a
      //          temporary "Get it spinning" kind of thing, to be deprecated in favour of FOC
      //        BLDCCurrentController();
      //        BLDCCommuteHall();
      //      }//For now we are going to not support BLDC mode
    	generateEnable();
      if (MotorSensorMode == MOTOR_SENSOR_MODE_HALL) {
        hallAngleEstimator();
        angleObserver();
        MESCFOC();
        writePWM();
      } else if (MotorSensorMode == MOTOR_SENSOR_MODE_SENSORLESS) {
        flux_observer();
        MESCFOC();
        writePWM();
      }
      break;

    case MOTOR_STATE_TRACKING:
#ifdef HAS_PHASE_SENSORS
			  // Track using BEMF from phase sensors
			  generateBreak();
		      MESCTrack();
		      flux_observer();
#endif

      break;

    case MOTOR_STATE_OPEN_LOOP_STARTUP:
      // Same as open loop
      OLGenerateAngle();
      MESCFOC();
      writePWM();
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
        generateBreak();
      // Do basically nothing
      break;

    case MOTOR_STATE_DETECTING:

      if ((current_hall_state == 7)) {
        // no hall sensors detected, all GPIO pulled high
        MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
        MotorState = MOTOR_STATE_GET_KV;
      } else if (current_hall_state == 0) {
        MotorState = MOTOR_STATE_ERROR;
        MotorError = MOTOR_ERROR_HALL0;
      }
      // ToDo add reporting
      else {
        // hall sensors detected
        MotorSensorMode = MOTOR_SENSOR_MODE_HALL;
        getHallTable();
        MESCFOC();
        writePWM();
      }
      break;

    case MOTOR_STATE_MEASURING:

//      if (motor.uncertainty ==
//          1) {  // Every PWM cycle we enter this function until
                // the resistance measurement has converged at a
                // good value. Once the measurement is complete,
                // Rphase is set, and this is no longer called
        if (foc_vars.initing == 0) {
          measureResistance();
        }
        break;
//      }
//        else if (motor.Lphase == 0)  // This is currently rolled into measureResistance() since
//                     // it seemed pointless to re-write basically the same
//                     // function...
//      {
//        // As per resistance measurement, this will be called until an
//        // inductance measurement is converged.
//        // measureInductance();
//        break;
//      }

      break;

    case MOTOR_STATE_GET_KV:
      getkV();

      break;

    case MOTOR_STATE_ERROR:
      generateBreak();  // Generate a break state (software disabling all PWM
                        // phases, hardware OVCP reserved for fatal situations
                        // requiring reset)
                        // Now panic and freak out
      break;

    case MOTOR_STATE_ALIGN:
      // Turn on at a given voltage at electricalangle0;
      break;
    case MOTOR_STATE_TEST:
      // Double pulse test
      doublePulseTest();
      break;
    case MOTOR_STATE_RECOVERING:

	      deadshort(); //Function to startup motor from running without phase sensors

      break;
    default:
      MotorState = MOTOR_STATE_ERROR;
      generateBreak();
      break;
  }
#ifdef SOFTWARE_ADC_REGULAR
        HAL_ADC_Start(&hadc1); //ToDo Eliminate the HAL call, slow and inefficient.
#endif
}

// The hyperloop runs at PWM timer bottom, when the PWM is in V7 (all high)
// In this loop, we write the values of the PWM to be updated at the next update
// event (timer top) This is where we want to inject signals for measurement so
// that the next signal level takes affect right after the ADC reading In normal
// run mode, we want to increment the angle and write the next PWM values
static float Idq[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
static float dIdq[2] = {0.0f, 0.0f};
//static float IIR[2] = {0.0f, 0.0f};
static float intdidq[2];
static volatile float nrm;
static volatile float nrm_avg;

void hyperLoop() {
#ifdef USE_HFI
  if (foc_vars.inject) {
    if (foc_vars.inject_high_low_now == 0) {
      foc_vars.inject_high_low_now = 1;
      foc_vars.Vdq[0] = foc_vars.Vdq[0] + foc_vars.Vd_injectionV;
      foc_vars.Vdq[1] = foc_vars.Vdq[1] + foc_vars.Vq_injectionV;
      Idq[0][0] = foc_vars.Idq[0];
      Idq[0][1] = foc_vars.Idq[1];
    } else if (foc_vars.inject_high_low_now == 1) {
      foc_vars.inject_high_low_now = 0;
      foc_vars.Vdq[0] = foc_vars.Vdq[0] - foc_vars.Vd_injectionV;
      foc_vars.Vdq[1] = foc_vars.Vdq[1] - foc_vars.Vq_injectionV;
      Idq[1][0] = foc_vars.Idq[0];
      Idq[1][1] = foc_vars.Idq[1];
    }
    if (MotorState == MOTOR_STATE_RUN) {


  	  dIdq[0] = (Idq[0][0] - Idq[1][0]);
  	  dIdq[1] = (Idq[0][1] - Idq[1][1]);
  	  if(dIdq[1]>1.0f){dIdq[1] = 1.0f;}
  	  if(dIdq[1]<-1.0f){dIdq[1] = -1.0f;}
  	  intdidq[1] = (intdidq[1] + dIdq[1]);
  	  if(intdidq[1]>10){intdidq[1]=10;}
  	  if(intdidq[1]<-10){intdidq[1]=-10;}

  	  static float ffactor = 2.0f;

		foc_vars.IIR[0] *= (99.0f);
		foc_vars.IIR[1] *= (1.0f);

		foc_vars.IIR[0] += dIdq[0];
		foc_vars.IIR[1] += dIdq[1];

		foc_vars.IIR[0] *= 0.01f;
		foc_vars.IIR[1] *=0.5f;
        foc_vars.FOCAngle += (int)(250.0f*foc_vars.IIR[1] + 5.50f*intdidq[1]);
    }
  }
#endif
#ifdef USE_ENCODER
tle5012();
#endif
 // foc_vars.FOCAngle = foc_vars.FOCAngle + foc_vars.angle_error;
if(MotorState==MOTOR_STATE_RUN||MotorState==MOTOR_STATE_MEASURING){
	writePWM();
	}
}

#define MAX_ERROR_COUNT 2

// TODO: refactor this function. Is this function called by DMA interrupt?
void VICheck() {  // Check currents, voltages are within panic limits
  static int errorCount = 0;

  if ((measurement_buffers.RawADC[0][FOC_CHANNEL_PHASE_I] > g_hw_setup.RawCurrLim) ||
      (measurement_buffers.RawADC[1][FOC_CHANNEL_PHASE_I] > g_hw_setup.RawCurrLim) ||
      (measurement_buffers.RawADC[2][FOC_CHANNEL_PHASE_I] > g_hw_setup.RawCurrLim) ||
      (measurement_buffers.RawADC[0][FOC_CHANNEL_DC_V   ] > g_hw_setup.RawVoltLim)){
        foc_vars.Idq_req[0] = foc_vars.Idq_req[0] * 0.9;
        foc_vars.Idq_req[1] = foc_vars.Idq_req[1] * 0.9;

        errorCount++;
        if (errorCount >= MAX_ERROR_COUNT) {
          generateBreak();
          measurement_buffers.adc1 = measurement_buffers.RawADC[0][FOC_CHANNEL_PHASE_I];
          measurement_buffers.adc2 = measurement_buffers.RawADC[1][FOC_CHANNEL_PHASE_I];
          measurement_buffers.adc3 = measurement_buffers.RawADC[2][FOC_CHANNEL_PHASE_I];
          measurement_buffers.adc4 = measurement_buffers.RawADC[0][FOC_CHANNEL_DC_V   ];

          MotorState = MOTOR_STATE_ERROR;
          MotorError = MOTOR_ERROR_OVER_LIMIT;
        }
      }
      else {
        errorCount = 0;
      }
  }

  void ADCConversion() {
	  foc_vars.Vdq_smoothed[0] = (foc_vars.Vdq_smoothed[0]*99.0f + foc_vars.Vdq[0])*0.01f;
	  foc_vars.Vdq_smoothed[1] = (foc_vars.Vdq_smoothed[1]*99.0f + foc_vars.Vdq[1])*0.01f;
	  foc_vars.Idq_smoothed[0] = (foc_vars.Idq_smoothed[0]*99.0f + foc_vars.Idq[0])*0.01f;
	  foc_vars.Idq_smoothed[1] = (foc_vars.Idq_smoothed[1]*99.0f + foc_vars.Idq[1])*0.01f;

    getRawADC();
    VICheck();

    // Here we take the raw ADC values, offset, cast to (float) and use the
    // hardware gain values to create volt and amp variables

    if (foc_vars.initing) {
    	MotorState = MOTOR_STATE_TRACKING;
      for (uint32_t i = 0; i < 3; i++) {
        measurement_buffers.ADCOffset[i] += measurement_buffers.RawADC[i][FOC_CHANNEL_PHASE_I];
      }

      static int initcycles = 0;
      initcycles = initcycles + 1;
      if (initcycles == 1000) {
        calculateGains();
        calculateVoltageGain();
        flux_linked_beta = 0.001;
        flux_linked_alpha = 0.001;
        for (uint32_t i = 0; i < 3; i++) {
          measurement_buffers.ADCOffset[i] /= 1000;
        }
        htim1.Instance->BDTR |= TIM_BDTR_MOE;
        foc_vars.initing = 0;
      }
    }
//Convert the currents to real amps in SI units
    measurement_buffers.ConvertedADC[0][FOC_CHANNEL_PHASE_I] =
        (float)(measurement_buffers.RawADC[0][FOC_CHANNEL_PHASE_I] -
                measurement_buffers.ADCOffset[0]) *
        g_hw_setup.Igain;
    measurement_buffers.ConvertedADC[1][0] =
        (float)(measurement_buffers.RawADC[1][0] -
                measurement_buffers.ADCOffset[1]) *
        g_hw_setup.Igain;
    measurement_buffers.ConvertedADC[2][FOC_CHANNEL_PHASE_I] =
        (float)(measurement_buffers.RawADC[2][FOC_CHANNEL_PHASE_I] -
                measurement_buffers.ADCOffset[2]) *
        g_hw_setup.Igain;
//Convert the voltages to volts in real SI units
    measurement_buffers.ConvertedADC[0][1] =
        (float)measurement_buffers.RawADC[0][1] * g_hw_setup.VBGain;  // Vbus
    measurement_buffers.ConvertedADC[0][2] =
        (float)measurement_buffers.RawADC[0][2] * g_hw_setup.VBGain;  // Usw
    measurement_buffers.ConvertedADC[1][1] =
        (float)measurement_buffers.RawADC[1][1] * g_hw_setup.VBGain;  // Vsw
    measurement_buffers.ConvertedADC[1][2] =
        (float)measurement_buffers.RawADC[1][2] * g_hw_setup.VBGain;  // Wsw

//Deal with terrible hardware choice of only having two current sensors
//Based on Iu+Iv+Iw = 0
#ifdef MISSING_UCURRSENSOR
    measurement_buffers.ConvertedADC[0][FOC_CHANNEL_PHASE_I] =
    		-measurement_buffers.ConvertedADC[1][FOC_CHANNEL_PHASE_I] - measurement_buffers.ConvertedADC[2][FOC_CHANNEL_PHASE_I];
#endif
#ifdef MISSING_VCURRSENSOR
    measurement_buffers.ConvertedADC[1][FOC_CHANNEL_PHASE_I] =
    		-measurement_buffers.ConvertedADC[0][FOC_CHANNEL_PHASE_I] - measurement_buffers.ConvertedADC[2][FOC_CHANNEL_PHASE_I];
#endif
#ifdef MISSING_WCURRSENSOR
    measurement_buffers.ConvertedADC[2][FOC_CHANNEL_PHASE_I] =
    		-measurement_buffers.ConvertedADC[0][FOC_CHANNEL_PHASE_I] - measurement_buffers.ConvertedADC[1][FOC_CHANNEL_PHASE_I];
#endif
    // Power Variant Clark transform
    // Here we select the phases that have the lowest duty cycle to us, since
    // they should have the best current measurements
    if (htim1.Instance->CCR1 > foc_vars.ADC_duty_threshold) {
      // Clark using phase V and W
      foc_vars.Iab[0] = -measurement_buffers.ConvertedADC[ADCIV][FOC_CHANNEL_PHASE_I] -
                        measurement_buffers.ConvertedADC[ADCIW][FOC_CHANNEL_PHASE_I];
      foc_vars.Iab[1] =
          one_on_sqrt3 * measurement_buffers.ConvertedADC[ADCIV][FOC_CHANNEL_PHASE_I] -
          one_on_sqrt3 * measurement_buffers.ConvertedADC[ADCIW][FOC_CHANNEL_PHASE_I];
    } else if (htim1.Instance->CCR2 > foc_vars.ADC_duty_threshold) {
      // Clark using phase U and W
      foc_vars.Iab[0] = measurement_buffers.ConvertedADC[ADCIU][FOC_CHANNEL_PHASE_I];
      foc_vars.Iab[1] =
          -one_on_sqrt3 * measurement_buffers.ConvertedADC[ADCIU][FOC_CHANNEL_PHASE_I] -
          two_on_sqrt3 * measurement_buffers.ConvertedADC[ADCIW][FOC_CHANNEL_PHASE_I];
    } else if (htim1.Instance->CCR3 > foc_vars.ADC_duty_threshold) {
      //        measurement_buffers.ConvertedADC[ADCIW][FOC_CHANNEL_PHASE_I] =
      //        			-
      //        measurement_buffers.ConvertedADC[ADCIU][FOC_CHANNEL_PHASE_I] -
      //    				measurement_buffers.ConvertedADC[ADCIV][FOC_CHANNEL_PHASE_I];
      // Clark using phase U and V
      foc_vars.Iab[0] = measurement_buffers.ConvertedADC[ADCIU][FOC_CHANNEL_PHASE_I];
      foc_vars.Iab[1] =
          two_on_sqrt3 * measurement_buffers.ConvertedADC[ADCIV][FOC_CHANNEL_PHASE_I] +
          one_on_sqrt3 * two_on_sqrt3 *
              measurement_buffers.ConvertedADC[ADCIU][FOC_CHANNEL_PHASE_I];
    } else {
      // Do the full transform
      foc_vars.Iab[0] =
          0.66666f * measurement_buffers.ConvertedADC[ADCIU][FOC_CHANNEL_PHASE_I] -
          0.33333f * measurement_buffers.ConvertedADC[ADCIV][FOC_CHANNEL_PHASE_I] -
          0.33333f * measurement_buffers.ConvertedADC[ADCIW][FOC_CHANNEL_PHASE_I];
      foc_vars.Iab[1] =
          one_on_sqrt3 * measurement_buffers.ConvertedADC[ADCIV][FOC_CHANNEL_PHASE_I] -
          one_on_sqrt3 * measurement_buffers.ConvertedADC[ADCIW][FOC_CHANNEL_PHASE_I];
    }

    // Park
    foc_vars.Idq[0] = foc_vars.sincosangle.cos * foc_vars.Iab[0] +
                      foc_vars.sincosangle.sin * foc_vars.Iab[1];
    foc_vars.Idq[1] = foc_vars.sincosangle.cos * foc_vars.Iab[1] -
                      foc_vars.sincosangle.sin * foc_vars.Iab[0];
  }
  /////////////////////////////////////////////////////////////////////////////
  // SENSORLESS IMPLEMENTATION//////////////////////////////////////////////////
  static float Ia_last = 0.0f;
  static float Ib_last = 0.0f;
  static uint16_t angle = 0;
  volatile static float FLAnow = 0.0f;
  volatile static float FLAmax = 0.0f;
  volatile static float FLAmin = 0.0f;
  volatile static float FLAdiff = 0.0f;
static int cyclescount = 0;
static int cyclescountacc = 0;

  void flux_observer() {
    // LICENCE NOTE:
    // This function deviates slightly from the BSD 3 clause licence.
    // The work here is entirely original to the MESC FOC project, and not based
    // on any appnotes, or borrowed from another project. This work is free to
    // use, as granted in BSD 3 clause, with the exception that this note must
    // be included in where this code is implemented/modified to use your
    // variable names, structures containing variables or other minor
    // rearrangements in place of the original names I have chosen, and credit
    // to David Molony as the original author must be noted.

    // With thanks to C0d3b453 for generally keeping this compiling and Elwin
    // for producing data comparing the output to a 16bit encoder.


#if 0// Code for investigating the flux linkage live. Initially used for Field weakening investigation
	  if(foc_vars.FOCAngle<500){
		  cyclescountacc = cyclescount;
		  cyclescount = 0;
		  foc_vars.FLAdiff = 0.5f*(FLAmax-FLAmin);//This is the flux linkage of the last electrical cycle
		  FLAnow = 0.0f;
		  FLAmax = 0.0f;
		  FLAmin = 0.0f;
	  }
	  cyclescount++;
	  FLAnow = FLAnow + (foc_vars.Vab[0] - motor.Rphase * foc_vars.Iab[0])*foc_vars.pwm_period-
		        motor.Lphase * (foc_vars.Iab[0] - Ia_last);
	  if(FLAnow>FLAmax){FLAmax = FLAnow;}
	  if(FLAnow<FLAmin){FLAmin = FLAnow;}
#endif

#ifdef USE_FLUX_LINKAGE_OBSERVER
	  //Variant of the flux linkage observer created by/with Benjamin Vedder to
	  //eliminate the need to accurately know the flux linked motor parameter.
	  //This may be useful when approaching saturation; currently unclear but
	  //definitely makes setup less critical.
	  //It basically takes the normal of the flux linkage at any time and
	  //changes the flux limits accordingly, ignoring using a sqrt for computational efficiency
	  float flux_linked_norm = flux_linked_alpha*flux_linked_alpha+flux_linked_beta*flux_linked_beta;
	  float flux_err = flux_linked_norm-motor_profile->flux_linkage*motor_profile->flux_linkage;
	  motor_profile->flux_linkage = motor_profile->flux_linkage+ motor_profile->flux_linkage_gain*flux_err;
	  if(motor_profile->flux_linkage>motor_profile->flux_linkage_max){motor_profile->flux_linkage = motor_profile->flux_linkage_max;}
	  if(motor_profile->flux_linkage<motor_profile->flux_linkage_min){motor_profile->flux_linkage = motor_profile->flux_linkage_min;}
#endif
	// This is the actual observer function.
	// We are going to integrate Va-Ri and clamp it positively and negatively
	// the angle is then the arctangent of the integrals shifted 180 degrees
    flux_linked_alpha =
        flux_linked_alpha + (foc_vars.Vab[0] - motor_profile->R * foc_vars.Iab[0])*foc_vars.pwm_period-
		motor_profile->L_D * (foc_vars.Iab[0] - Ia_last);
    flux_linked_beta =
        flux_linked_beta + (foc_vars.Vab[1] - motor_profile->R * foc_vars.Iab[1])*foc_vars.pwm_period -
		motor_profile->L_D * (foc_vars.Iab[1] - Ib_last);
    Ia_last = foc_vars.Iab[0];
    Ib_last = foc_vars.Iab[1];

#ifdef USE_NONLINEAR_OBSERVER_CENTERING
///Try directly applying the centering using the same method as the flux linkage observer
    float err = motor_profile->flux_linkage*motor_profile->flux_linkage-flux_linked_alpha*flux_linked_alpha-flux_linked_beta*flux_linked_beta;
    flux_linked_beta = flux_linked_beta+err*flux_linked_beta*motor_profile->non_linear_centering_gain;
    flux_linked_alpha = flux_linked_alpha+err*flux_linked_alpha*motor_profile->non_linear_centering_gain;
#endif
#ifdef USE_CLAMPED_OBSERVER_CENTERING
    if (flux_linked_alpha > motor_profile->flux_linkage) {
      flux_linked_alpha = motor_profile->flux_linkage;}
    if (flux_linked_alpha < -motor_profile->flux_linkage) {
      flux_linked_alpha = -motor_profile->flux_linkage;}
    if (flux_linked_beta > motor_profile->flux_linkage) {
      flux_linked_beta = motor_profile->flux_linkage;}
    if (flux_linked_beta < -motor_profile->flux_linkage) {
      flux_linked_beta = -motor_profile->flux_linkage;}
#endif
    angle = (uint16_t)(32768.0f + 10430.0f * fast_atan2(flux_linked_beta, flux_linked_alpha)) - 32768;

    //Run PLL for speed
    foc_vars.angle_error = foc_vars.angle_error-0.1f*(int16_t)((foc_vars.angle_error+(int)(foc_vars.FOCAngle - angle)));
    foc_vars.eHz = foc_vars.angle_error * foc_vars.pwm_frequency/65536.0f;
    if(foc_vars.inject==0){
    foc_vars.FOCAngle = angle;
    }

#ifdef USE_ENCODER
    foc_vars.enc_obs_angle = foc_vars.enc_angle-foc_vars.FOCAngle;
    //foc_vars.FOCAngle = foc_vars.enc_angle;
    //foc_vars.FOCAngle = 0; //for aligning encoder for testing
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
    float abs_y = fabs(y);
    float abs_x = fabs(x);
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
  void hallAngleEstimator() {  // Implementation using the mid point of the hall
                               // sensor angles, which should be much more
                               // reliable to generate that the edges

    if (current_hall_state != last_hall_state) {
      foc_vars.hall_update = 1;
      if (current_hall_state == 0) {
        MotorState = MOTOR_STATE_ERROR;
        MotorError = MOTOR_ERROR_HALL0;
      } else if (current_hall_state == 7) {
        MotorState = MOTOR_STATE_ERROR;
        MotorError = MOTOR_ERROR_HALL7;
      }
      //////////Implement the Hall table here, but the vector can be dynamically
      /// created/filled by another function/////////////
      current_hall_angle = motor_profile->sensor.hall_states[current_hall_state - 1].mid;

      // Calculate Hall error

      uint16_t a;
      if ((a = current_hall_angle - last_hall_angle) < 32000)  // Forwards
      {
        hall_error =
            foc_vars.FOCAngle -  motor_profile->sensor.hall_states[current_hall_state - 1].min;
        dir = 1.0f;
        // foc_vars.HallAngle = foc_vars.HallAngle - 5460;
      } else  // Backwards
      {
        hall_error =
            foc_vars.FOCAngle -  motor_profile->sensor.hall_states[current_hall_state - 1].max;
        dir = -1.0f;
        // foc_vars.HallAngle = foc_vars.HallAngle + 5460;
      }
      if (hall_error > 32000) {
        hall_error = hall_error - 65536;
      }
      if (hall_error < -32000) {
        hall_error = hall_error + 65536;
      }
    }
  }

  void angleObserver() {
    // This function should take the available data (hall change, BEMF crossing
    // etc...) and process it with a PLL type mechanism
    if (foc_vars.hall_update == 1) {
      foc_vars.hall_update = 0;
      last_observer_period = ticks_since_last_observer_change;
      float one_on_ticks = (1.0 / ticks_since_last_observer_change);
      one_on_last_observer_period =
          (4 * one_on_last_observer_period + (one_on_ticks)) * 0.2;  // ;
      angle_step =
          (4 * angle_step +
           (one_on_ticks)* motor_profile->sensor.hall_states[last_hall_state - 1].width) *
          0.2;

      // Reset the counters, track the previous state
      last_hall_state = current_hall_state;
      last_hall_angle = current_hall_angle;
      ticks_since_last_observer_change = 0;
    }

    // Run the counter
    ticks_since_last_observer_change = ticks_since_last_observer_change + 1;

    if (ticks_since_last_observer_change <= 2.0 * last_observer_period) {
      /*      foc_vars.FOCAngle = foc_vars.FOCAngle + (uint16_t)(dir*angle_step
         + one_on_last_observer_period * (-0.9 * hall_error)); //Does not
         work...
           //Why?
 */
      if (dir > 0) {  // Apply a gain to the error as well as the feed forward
        // from the last hall period. Gain of 0.9-1.1 seems to work
        // well when using corrected hall positions and spacings
        foc_vars.FOCAngle =
            foc_vars.FOCAngle +
            (uint16_t)(angle_step - one_on_last_observer_period * hall_error);
        // one_on_last_observer_period * (-0.2 * hall_error));
      } else if (dir < 0) {
        foc_vars.FOCAngle =
            foc_vars.FOCAngle +
            (uint16_t)(-angle_step +
                       one_on_last_observer_period * (-0.9 * hall_error));
        // Also does not work,
        // Why??
        foc_vars.FOCAngle =
            foc_vars.FOCAngle -
            (uint16_t)(angle_step +
                       one_on_last_observer_period * (0.2 * hall_error));
      }
    }
    if (ticks_since_last_observer_change > 1500.0f) {
      ticks_since_last_observer_change = 1500.0f;
      last_observer_period = 1500.0f;  //(ticks_since_last_hall_change);
      one_on_last_observer_period =
          1.0f / last_observer_period;  // / ticks_since_last_hall_change;
      foc_vars.FOCAngle = current_hall_angle;
    }
  }

  void OLGenerateAngle() {
    foc_vars.openloop_step = 50;

    foc_vars.FOCAngle = foc_vars.FOCAngle + foc_vars.openloop_step;
    // ToDo
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // FOC PID algorithms
  //////////////////////////////////////////////////////////////////////////////////////////

  void MESCFOC() {
#ifdef USE_FIELD_WEAKENING
	    if (fabs(foc_vars.Vdq[1]) > foc_vars.field_weakening_threshold) {
	      foc_vars.Idq_req[0] =
	          foc_vars.field_weakening_curr_max *foc_vars.field_weakening_multiplier*
	          (foc_vars.field_weakening_threshold - fabs(foc_vars.Vdq_smoothed[1]));
	      foc_vars.field_weakening_flag = 1;
	    } else {
	    	//if(!foc_vars.inject){
	      //foc_vars.Idq_req[0] = 0;}
	      foc_vars.field_weakening_flag = 0;
	    }
#endif
    // Here we are going to do a PID loop to control the dq currents, converting
    // Idq into Vdq Calculate the errors
    static float Idq_err[2];
    Idq_err[0] = (foc_vars.Idq_req[0] - foc_vars.Idq[0]) * foc_vars.Id_pgain;
    Idq_err[1] = (foc_vars.Idq_req[1] - foc_vars.Idq[1]) * foc_vars.Iq_pgain;

    // Integral error
    foc_vars.Idq_int_err[0] =
    		foc_vars.Idq_int_err[0] + foc_vars.Id_igain * Idq_err[0] * foc_vars.pwm_period;
    foc_vars.Idq_int_err[1] =
    		foc_vars.Idq_int_err[1] + foc_vars.Iq_igain * Idq_err[1] * foc_vars.pwm_period;
    // Apply the integral gain at this stage to enable bounding it

    static int i = 0;
    if (i == 0) {  // set or release the PID controller; may want to do this for
                   // cycle skipping, which may help for high inductance motors
      // Bounding

        if (foc_vars.Idq_int_err[0] > foc_vars.Vdint_max){foc_vars.Idq_int_err[0] = foc_vars.Vdint_max;}
        if (foc_vars.Idq_int_err[0] < -foc_vars.Vdint_max){foc_vars.Idq_int_err[0] = -foc_vars.Vdint_max;}
        if (foc_vars.Idq_int_err[1] > foc_vars.Vqint_max){foc_vars.Idq_int_err[1] = foc_vars.Vqint_max;}
        if (foc_vars.Idq_int_err[1] < -foc_vars.Vqint_max){foc_vars.Idq_int_err[1] = -foc_vars.Vqint_max;}

        // Apply the PID, and potentially smooth the output for noise - sudden
      // changes in VDVQ may be undesirable for some motors. Integral error is
      // pre-bounded to avoid integral windup, proportional gain needs to have
      // effect even at max integral to stabilise and avoid trips
      foc_vars.Vdq[0] = Idq_err[0] + foc_vars.Idq_int_err[0];
      foc_vars.Vdq[1] = Idq_err[1] + foc_vars.Idq_int_err[1];

      // Bounding final output

#ifdef USE_SQRT_CIRCLE_LIM
      float Vmagnow2 = foc_vars.Vdq[0]*foc_vars.Vdq[0]+foc_vars.Vdq[1]*foc_vars.Vdq[1];
      //Check if the vector length is greater than the available voltage
      if(Vmagnow2>foc_vars.Vmag_max2){
      float Vmagnow = sqrtf(Vmagnow2);
      float one_on_Vmagnow = 1/Vmagnow;
      foc_vars.Vdq[0] = foc_vars.Vdq[0]*foc_vars.Vmag_max*one_on_Vmagnow;
      foc_vars.Vdq[1] = foc_vars.Vdq[1]*foc_vars.Vmag_max*one_on_Vmagnow;
      }
#else
      // These limits are experimental, but result in close to 100% modulation.
      // Since Vd and Vq are orthogonal, limiting Vd is not especially helpful
      // in reducing overall voltage magnitude, since the relation
      // Vout=(Vd^2+Vq^2)^0.5 results in Vd having a small effect. Vd is
      // primarily used to drive the resistive part of the field; there is no
      // BEMF pushing against Vd and so it does not scale with RPM (except for
      // cross coupling).

      if (foc_vars.Vdq[0] > foc_vars.Vd_max)
        (foc_vars.Vdq[0] = foc_vars.Vd_max);
      if (foc_vars.Vdq[0] < -foc_vars.Vd_max)
        (foc_vars.Vdq[0] = -foc_vars.Vd_max);
      if (foc_vars.Vdq[1] > foc_vars.Vq_max)
        (foc_vars.Vdq[1] = foc_vars.Vq_max);
      if (foc_vars.Vdq[1] < -foc_vars.Vq_max)
        (foc_vars.Vdq[1] = -foc_vars.Vq_max);
#endif
      i = FOC_PERIODS;


    }
    i = i - 1;
//    if (foc_vars.inject) {
//      if (foc_vars.inject_high_low_now == 0) {
//        foc_vars.Vdq[0] = foc_vars.Vdq[0] + foc_vars.Vd_injectionV;
//        foc_vars.Vdq[1] = foc_vars.Vdq[1] + foc_vars.Vq_injectionV;
//        Idq[0][0] = foc_vars.Idq[0];
//        Idq[0][1] = foc_vars.Idq[1];
//      } else if (foc_vars.inject_high_low_now == 1) {
//        foc_vars.Vdq[0] = foc_vars.Vdq[0] - foc_vars.Vd_injectionV;
//        foc_vars.Vdq[1] = foc_vars.Vdq[1] - foc_vars.Vq_injectionV;
//        Idq[1][0] = foc_vars.Idq[0];
//        Idq[1][1] = foc_vars.Idq[1];
//      }
//    }
  }

  static float mid_value = 0;

  float top_value;
  float bottom_value;

  void writePWM() {
    // Now we update the sin and cos values, since when we do the inverse
    // transforms, we would like to use the most up to date versions(or even the
    // next predicted version...)
	sin_cos_fast(foc_vars.FOCAngle, &foc_vars.sincosangle.sin, &foc_vars.sincosangle.cos);

    // Inverse Park transform
    foc_vars.Vab[0] = foc_vars.sincosangle.cos * foc_vars.Vdq[0] -
                      foc_vars.sincosangle.sin * foc_vars.Vdq[1];
    foc_vars.Vab[1] = foc_vars.sincosangle.sin * foc_vars.Vdq[0] +
                      foc_vars.sincosangle.cos * foc_vars.Vdq[1];
    foc_vars.Vab[2] = 0.0f;

	    // Inverse Clark transform - power variant
	  	foc_vars.inverterVoltage[0] = foc_vars.Vab[0];
	  	foc_vars.inverterVoltage[1] = -0.5f*foc_vars.inverterVoltage[0];
	  	foc_vars.inverterVoltage[2] = foc_vars.inverterVoltage[1] - sqrt3_on_2 * foc_vars.Vab[1];
	  	foc_vars.inverterVoltage[1] = foc_vars.inverterVoltage[1] + sqrt3_on_2 * foc_vars.Vab[1];

    ////////////////////////////////////////////////////////
    // SVPM implementation
    // Try to do this as a "midpoint clamp" where rather than finding the
    // lowest, we find the highest and lowest and subtract the middle
    top_value = foc_vars.inverterVoltage[0];
    bottom_value = top_value;

    if (foc_vars.inverterVoltage[1] > top_value) {
      top_value = foc_vars.inverterVoltage[1];
    }
    if (foc_vars.inverterVoltage[2] > top_value) {
      top_value = foc_vars.inverterVoltage[2];
    }
    if (foc_vars.inverterVoltage[1] < bottom_value) {
      bottom_value = foc_vars.inverterVoltage[1];
    }
    if (foc_vars.inverterVoltage[2] < bottom_value) {
      bottom_value = foc_vars.inverterVoltage[2];
    }

    mid_value = foc_vars.PWMmid -
                0.5f * foc_vars.Vab_to_PWM * (top_value + bottom_value);

    ////////////////////////////////////////////////////////
    // Actually write the value to the timer registers
    htim1.Instance->CCR1 = (uint16_t)(
        foc_vars.Vab_to_PWM * foc_vars.inverterVoltage[0] + mid_value);
    htim1.Instance->CCR2 = (uint16_t)(
        foc_vars.Vab_to_PWM * foc_vars.inverterVoltage[1] + mid_value);
    htim1.Instance->CCR3 = (uint16_t)(
        foc_vars.Vab_to_PWM * foc_vars.inverterVoltage[2] + mid_value);
  }

  // Here we set all the PWMoutputs to LOW, without triggering the timerBRK,
  // which should only be set by the hardware comparators, in the case of a
  // shoot-through or other catastrophic event This function means that the
  // timer can be left running, ADCs sampling etc which enables a recovery, or
  // single PWM period break in which the backEMF can be measured directly
  // This function needs implementing and testing before any high current or
  // voltage is applied, otherwise... DeadFETs
  void generateBreak() {
    phU_Break();
    phV_Break();
    phW_Break();
  }
  void generateEnable() {
    phU_Enable();
    phV_Enable();
    phW_Enable();
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

  void measureResistance() {
    static int PWM_cycles = 0;

    if (PWM_cycles < 2) {
      uint16_t half_ARR = htim1.Instance->ARR / 2;
      htim1.Instance->CCR1 = half_ARR;
      htim1.Instance->CCR2 = half_ARR;
      htim1.Instance->CCR3 = half_ARR;
      motor_profile->R = 0.001f;     // Initialise with a very low value 1mR
      motor_profile->L_D = 0.000001f;  // Initialise with a very low value 1uH
      motor_profile->L_Q = 0.000001f;
      calculateVoltageGain();    // Set initial gains to enable MESCFOC to run
      calculateGains();
      phU_Enable();
      phV_Enable();
      phW_Enable();
      foc_vars.Idq_req[0] = I_MEASURE;
      foc_vars.Idq_req[1] = 0.0f;
      foc_vars.FOCAngle = 0;

      foc_vars.inject = 0;  // flag to not inject at SVPWM top

      MESCFOC();
      writePWM();

      count_top = 0;
      count_bottom = 0;
    }

    else if (PWM_cycles < 35000) {  // Align the rotor for ~1 second
      foc_vars.Idq_req[0] = I_MEASURE;
      foc_vars.Idq_req[1] = 0.0f;

      foc_vars.inject = 0;
      MESCFOC();
      writePWM();
    }

    else if (PWM_cycles < 40000) {  // Lower setpoint
      foc_vars.Idq_req[0] = 0.20f*I_MEASURE;
      foc_vars.inject = 0;
      MESCFOC();
      writePWM();

      bottom_V = bottom_V + foc_vars.Vdq[0];
      bottom_I = bottom_I + foc_vars.Idq[0];
      count_bottom++;
    }

    else if (PWM_cycles < 45000) {  // Upper setpoint stabilisation
      foc_vars.Idq_req[0] = I_MEASURE;
      foc_vars.inject = 0;
      MESCFOC();
      writePWM();

    }

    else if (PWM_cycles < 50000) {  // Upper setpoint
      foc_vars.Idq_req[0] = I_MEASURE;
      foc_vars.inject = 0;
      MESCFOC();
      writePWM();

      top_V = top_V + foc_vars.Vdq[0];
      top_I = top_I + foc_vars.Idq[0];
      count_top++;
    } else if (PWM_cycles < 50001) {  // Calculate R

      generateBreak();
      motor_profile->R = (top_V - bottom_V) / (top_I - bottom_I);

      //Initialise the variables for the next measurement
      Vd_temp = foc_vars.Vdq[0] * 0.1f;  // Store the voltage required for the high setpoint, to
                       	   	   	   	   	 // use as an offset for the inductance
      Vq_temp = 0;
      foc_vars.Vdq[1] = 0;
      count_top = 0;
      count_bottom = 0;
      top_I_L = 0;
      bottom_I_L = 0;
      generateEnable();
    }
/////////////////////////// Collect Ld variable//////////////////////////
    else if (PWM_cycles < 80001) {
      // generateBreak();
      foc_vars.inject = 1;  // flag to the SVPWM writer to inject at top
      foc_vars.Vd_injectionV = V_MEASURE;
      foc_vars.Vq_injectionV = 0.0f;
      foc_vars.Vdq[0] = Vd_temp;
      foc_vars.Vdq[1] = 0;

      if (foc_vars.inject_high_low_now == 1) {
        top_I_L = top_I_L + foc_vars.Idq[0];
        count_top++;
      } else if (foc_vars.inject_high_low_now == 0) {
        bottom_I_L = bottom_I_L + foc_vars.Idq[0];
        count_bottom++;
      }
    }

    else if (PWM_cycles < 80002) {
      generateBreak();
      motor_profile->L_D =
          fabs((foc_vars.Vd_injectionV) /
          ((top_I_L - bottom_I_L) / (count_top * foc_vars.pwm_period)));
      top_I_Lq = 0;
      bottom_I_Lq = 0;
      count_topq = 0;
      count_bottomq = 0;
      __NOP();  // Put a break point on it...
    } else if (PWM_cycles < 80003) {
      phU_Enable();
      phV_Enable();
      phW_Enable();

////////////////////////// Collect Lq variable//////////////////////////////
    } else if (PWM_cycles < 100003) {
      //			generateBreak();
      foc_vars.Vd_injectionV = 0.0f;
      foc_vars.Vq_injectionV = V_MEASURE;
      foc_vars.inject = 1;  // flag to the SVPWM writer to update at top
      foc_vars.Vdq[0] = Vd_temp;  // Vd_temp to keep it aligned with D axis
      foc_vars.Vdq[1] = 0;

      if (foc_vars.inject_high_low_now == 1) {
        top_I_Lq = top_I_Lq + foc_vars.Idq[1];
        count_topq++;
      } else if (foc_vars.inject_high_low_now == 0) {
        bottom_I_Lq = bottom_I_Lq + foc_vars.Idq[1];
        count_bottomq++;
      }
    }

    else {
      generateBreak();
      motor_profile->L_Q =
          fabs((foc_vars.Vq_injectionV) /
          ((top_I_Lq - bottom_I_Lq) / (count_top * foc_vars.pwm_period)));

      MotorState = MOTOR_STATE_IDLE;

      foc_vars.inject = 0;  // flag to the SVPWM writer stop injecting at top
      foc_vars.Vd_injectionV = HFI_VOLTAGE;
      foc_vars.Vq_injectionV = 0.0f;
      calculateGains();
      // MotorState = MOTOR_STATE_IDLE;  //
      MotorState = MOTOR_STATE_DETECTING;
      PWM_cycles = 0;
      phU_Enable();
      phV_Enable();
      phW_Enable();
    }
    PWM_cycles++;
  }


  void getHallTable() {
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
      lasthallstate = hallstate;
      (void)lasthallstate;
      firstturn = 0;
    }

    ////// Align the rotor////////////////////
    static uint16_t a = 65535;
    if (a)  // Align time
    {
      foc_vars.Idq_req[0] = 10;
      foc_vars.Idq_req[1] = 0;

      foc_vars.FOCAngle = 0;
      a = a - 1;
    } else {
      foc_vars.Idq_req[0] = 10;
      foc_vars.Idq_req[1] = 0;
      static int dir = 1;
      if (pwm_count < 65534) {
        if (foc_vars.FOCAngle < (anglestep)) {
          rollover = hallstate;
        }
        if ((foc_vars.FOCAngle < (30000)) &&
            (foc_vars.FOCAngle > (29000 - anglestep))) {
          rollover = 0;
        }
        lasthallstate = hallstate;
        if (rollover == hallstate) {
          hallangles[hallstate][0] =
              hallangles[hallstate][0] +
              (uint32_t)65535;  // Accumulate the angles through the sweep
        }

        foc_vars.FOCAngle =
            foc_vars.FOCAngle + anglestep;  // Increment the angle
        hallangles[hallstate][0] =
            hallangles[hallstate][0] +
            foc_vars.FOCAngle;       // Accumulate the angles through the sweep
        hallangles[hallstate][1]++;  // Accumulate the number of PWM pulses for
                                     // this hall state
        pwm_count = pwm_count + 1;
      } else if (pwm_count < 65535) {
        if (dir == 1) {
          dir = 0;
          rollover = 0;
        }
        if ((foc_vars.FOCAngle < (12000)) && (hallstate != last_hall_state)) {
          rollover = hallstate;
        }
        if ((foc_vars.FOCAngle < (65535)) &&
            (foc_vars.FOCAngle > (65535 - anglestep))) {
          rollover = 0;
        }
        lasthallstate = hallstate;
        if (rollover == hallstate) {
          hallangles[hallstate][0] =
              hallangles[hallstate][0] +
              (uint32_t)65535;  // Accumulate the angles through the sweep
        }

        foc_vars.FOCAngle =
            foc_vars.FOCAngle - anglestep;  // Increment the angle
        hallangles[hallstate][0] =
            hallangles[hallstate][0] +
            foc_vars.FOCAngle;       // Accumulate the angles through the sweep
        hallangles[hallstate][1]++;  // Accumulate the number of PWM pulses for
                                     // this hall state
        pwm_count = pwm_count + 1;
      }
    }
    if (pwm_count == 65535) {
      generateBreak();  // Debugging
      for (int i = 1; i < 7; i++) {
        hallangles[i][0] = hallangles[i][0] / hallangles[i][1];
        if (hallangles[i][0] > 65535) {
          hallangles[i][0] = hallangles[i][0] - 65535;
        }
      }
      for (int i = 0; i < 6; i++) {
             motor_profile->sensor.hall_states[i].mid = hallangles[i + 1][0];//This is the center angle of the hall state
             motor_profile->sensor.hall_states[i].width = hallangles[i + 1][1];//This is the width of the hall state
             motor_profile->sensor.hall_states[i].min =  motor_profile->sensor.hall_states[i].mid- motor_profile->sensor.hall_states[i].width/2;//This is the start angle of the hall state
             motor_profile->sensor.hall_states[i].max =  motor_profile->sensor.hall_states[i].mid+ motor_profile->sensor.hall_states[i].width/2;//This is the end angle of the hall state
      }
      MotorState = MOTOR_STATE_RUN;
      foc_vars.Idq_req[0] = 0;
      foc_vars.Idq_req[1] = 0;
      phU_Enable();
      phV_Enable();
      phW_Enable();
    }
  }

  void measureInductance()  // UNUSED, THIS HAS BEEN ROLLED INTO THE MEASURE
                            // RESISTANCE... no point in 2 functions really...
  {
    /*
     * In this function, we are going to run at a fixed duty cycle (perhaps as
     * determined by Measure Resistance?), pushing ~5A through the motor coils
     * (~100ADCcounts). We will then wait until steady state achieved... 1000
     * PWM cycles? before modulating CCR4, which triggers the ADC to capture
     * currents at at least 2 time points within the PWM cycle With this change
     * in current, and knowing R from previous measurement, we can calculate L
     * using L=Vdt/dI=IRdt/dI ToDo Actually do this... ToDo Determination of the
     * direct and quadrature inductances for MTPA in future?
     */
  }
  static float acc_da = 0.0f;
  static float acc_db = 0.0f;
  static float da;
  static float db;

  void getkV() {
    foc_vars.Idq_req[0] = 15.0f;  // 10A for the openloop spin
    foc_vars.Idq_req[1] = 0.0f;   // 10A for the openloop spin

    OLGenerateAngle();
    static int count = 0;
    static float BEMFaccumulator = 0.0f;
    static int cycles = 0;
    if (cycles < 65000) {
      // ramp openloop
    } else if (cycles < 128000) {
      count++;
      BEMFaccumulator += flux_linked_alpha;
    } else {
      // generateBreak();

      MotorState = MOTOR_STATE_RUN;
      motor_profile->flux_linkage = BEMFaccumulator / (2 * count);
      if (motor_profile->flux_linkage > 5.0f && motor_profile->flux_linkage < 5000.0f) {
        MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
      } else {
        MotorState = MOTOR_STATE_ERROR;
        generateBreak();
      }
    }
    MESCFOC();
    writePWM();

    cycles++;
    // Here we are going to integrate the back EMF and find the constant in
    // openloop Theory is that the integral over a half wave represents the flux
    // linkage. Have to account for the lead/lag, so we take the euclidian
    // normal of Va and Vb, which results in a value 2x higher than we want,
    // since integration should always have a +C term

    da = foc_vars.Vab[0] - motor_profile->R * foc_vars.Iab[0] -
    		motor_profile->L_D * (foc_vars.Iab[0] - Ia_last) * foc_vars.pwm_frequency;
    db = foc_vars.Vab[1] - motor_profile->R * foc_vars.Iab[1] -
    		motor_profile->L_D * (foc_vars.Iab[1] - Ib_last) * foc_vars.pwm_frequency;
    if (foc_vars.FOCAngle > 32468 && foc_vars.FOCAngle < 65035) {
      acc_da += da;
      acc_db += db;
    } else if (foc_vars.FOCAngle > 32468 &&
               foc_vars.FOCAngle < (65035 + foc_vars.openloop_step)) {
      flux_linked_alpha = sqrtf((acc_da * acc_da + acc_db * acc_db));
      acc_da = 0;
      acc_db = 0;
    }

    Ia_last = foc_vars.Iab[0];
    Ib_last = foc_vars.Iab[1];

    // MotorState = MOTOR_STATE_RUN;
  }

  // Temporary buffer which is used to turn on/off phase PWMs
  // Turn all phase U FETs off, Tristate the HBridge output - For BLDC mode
  // mainly, but also used for measuring, software fault detection and recovery
  // ToDo TEST THOROUGHLY The register manipulations for the break functions
  // were used previously on an STM32F042K6 for my first BLDC drive, on TIM1,
  // which should be identical, but definitely needs checking

  uint32_t tmpccmrx;

  void phU_Break() {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC1M;
    tmpccmrx &= ~TIM_CCMR1_CC1S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC1E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC1NE;  // disable
  }
  // Basically un-break phase U, opposite of above...
  void phU_Enable() {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC1M;
    tmpccmrx &= ~TIM_CCMR1_CC1S;
    tmpccmrx |= TIM_OCMODE_PWM1;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC1E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC1NE;  // enable
  }

  void phV_Break() {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC2M;
    tmpccmrx &= ~TIM_CCMR1_CC2S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE << 8;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC2E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC2NE;  // disable
  }

  void phV_Enable() {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC2M;
    tmpccmrx &= ~TIM_CCMR1_CC2S;
    tmpccmrx |= TIM_OCMODE_PWM1 << 8;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC2E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC2NE;  // enable
  }

  void phW_Break() {
    tmpccmrx = htim1.Instance->CCMR2;
    tmpccmrx &= ~TIM_CCMR2_OC3M;
    tmpccmrx &= ~TIM_CCMR2_CC3S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
    htim1.Instance->CCMR2 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC3E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC3NE;  // disable
  }

  void phW_Enable() {
    tmpccmrx = htim1.Instance->CCMR2;
    tmpccmrx &= ~TIM_CCMR2_OC3M;
    tmpccmrx &= ~TIM_CCMR2_CC3S;
    tmpccmrx |= TIM_OCMODE_PWM1;
    htim1.Instance->CCMR2 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC3E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC3NE;  // enable
  }

  void calculateGains() {
    foc_vars.pwm_frequency =PWM_FREQUENCY;
    foc_vars.pwm_period = 1.0f/foc_vars.pwm_frequency;
    htim1.Instance->ARR = HAL_RCC_GetHCLKFreq()/(((float)htim1.Instance->PSC + 1.0f) * 2*foc_vars.pwm_frequency);
    htim1.Instance->CCR4 = htim1.Instance->ARR-1;

    foc_vars.PWMmid = htim1.Instance->ARR * 0.5f;

    foc_vars.ADC_duty_threshold = htim1.Instance->ARR * 0.85f;

    foc_vars.Id_pgain = 10000.0f *motor_profile->L_D;  // 10000rads-1, hardcoded for now, * motorL

    foc_vars.Id_igain = motor_profile->R / motor_profile->L_D;
    // Pole zero cancellation for series PI control

    foc_vars.Iq_pgain = foc_vars.Id_pgain;
    foc_vars.Iq_igain = foc_vars.Id_igain;
//    foc_vars.Vdqres_to_Vdq = 0.333f * measurement_buffers.ConvertedADC[0][1] / 677.0f; //ToDo delete. Dud, old, I think?
    foc_vars.field_weakening_curr_max = FIELD_WEAKENING_CURRENT;  // test number, to be stored in user settings
    Lqd_diff = motor_profile->L_Q-motor_profile->L_D;
  }

  void calculateVoltageGain() {
    // We need a number to convert between Va Vb and raw PWM register values
    // This number should be the bus voltage divided by the ARR register
    foc_vars.Vab_to_PWM =
        htim1.Instance->ARR / measurement_buffers.ConvertedADC[0][1];
    // We also need a number to set the maximum voltage that can be effectively
    // used by the SVPWM This is equal to
    // 0.5*Vbus*MAX_MODULATION*SVPWM_MULTIPLIER*Vd_MAX_PROPORTION
    foc_vars.Vmag_max = 0.5f * measurement_buffers.ConvertedADC[0][1] *
            MAX_MODULATION * SVPWM_MULTIPLIER;
    foc_vars.Vmag_max2 = foc_vars.Vmag_max*foc_vars.Vmag_max;
    foc_vars.Vd_max = 0.5f * measurement_buffers.ConvertedADC[0][1] *
                      MAX_MODULATION * SVPWM_MULTIPLIER * Vd_MAX_PROPORTION;
    foc_vars.Vq_max = 0.5f * measurement_buffers.ConvertedADC[0][1] *
                      MAX_MODULATION * SVPWM_MULTIPLIER * Vq_MAX_PROPORTION;
#ifdef USE_SQRT_CIRCLE_LIM
    foc_vars.Vd_max = foc_vars.Vmag_max;
    foc_vars.Vq_max = foc_vars.Vmag_max;

#endif

    foc_vars.Vdint_max = foc_vars.Vd_max * 0.9f; //ToDo unvoodoo, logic in this is to always ensure headroom for the P term
    foc_vars.Vqint_max = foc_vars.Vq_max * 0.9f;

    foc_vars.field_weakening_threshold = foc_vars.Vq_max * FIELD_WEAKENING_THRESHOLD;
    foc_vars.field_weakening_multiplier = 1.0f-FIELD_WEAKENING_THRESHOLD;
  }

  void doublePulseTest() {
    static int dp_counter;
    static int dp_periods = 3;
    if (dp_counter < dp_periods) {
      htim1.Instance->CCR1 = 0;
      htim1.Instance->CCR2 = 0;
      htim1.Instance->CCR3 = 1022;
      phU_Break();
      phV_Enable();
      phW_Enable();
      test_vals.dp_current_final[dp_counter] =
          measurement_buffers.ConvertedADC[1][FOC_CHANNEL_PHASE_I];
      dp_counter++;
    } else if (dp_counter == dp_periods) {
      htim1.Instance->CCR2 = 0;
      htim1.Instance->CCR3 = 100;
      test_vals.dp_current_final[dp_counter] =
          measurement_buffers.ConvertedADC[1][FOC_CHANNEL_PHASE_I];
      dp_counter++;
    } else {
      htim1.Instance->CCR1 = 0;
      htim1.Instance->CCR2 = 0;
      htim1.Instance->CCR3 = 0;
      test_vals.dp_current_final[dp_counter] =
          measurement_buffers.ConvertedADC[1][FOC_CHANNEL_PHASE_I];
      dp_counter = 0;
      generateBreak();
      MotorState = MOTOR_STATE_IDLE;
    }
  }
  void MESC_Slow_IRQ_handler(TIM_HandleTypeDef *htim){


	  if(htim->Instance->SR & TIM_FLAG_CC2){
		  input_vars.IC_duration = htim->Instance->CCR1;// HAL_TIM_ReadCapturedValue(&htim4 /*&htim3*/, TIM_CHANNEL_1);
		  input_vars.IC_pulse = htim->Instance->CCR2;//HAL_TIM_ReadCapturedValue(&htim4 /*&htim3*/, TIM_CHANNEL_2);
		  input_vars.pulse_recieved = 1;

	  }else{
		  input_vars.IC_duration = 50000;
		  input_vars.IC_pulse = 0;
		  input_vars.pulse_recieved = 0;

	  }

	    if(htim->Instance->SR & TIM_FLAG_UPDATE){
	    		      slowLoop(htim);
	    }
  }
  extern uint32_t ADC_buffer[6];

  void slowLoop(TIM_HandleTypeDef * htim) {
    // The slow loop runs at either 20Hz or at the PWM input frequency.
    // In this loop, we will fetch the throttle values, and run functions that
    // are critical, but do not need to be executed very often e.g. adjustment
    // for battery voltage change
if(MotorState != MOTOR_STATE_MEASURING){
	  //Collect the requested throttle inputs
	  //UART input
	  if(0 == (input_vars.input_options & 0b1000)){
		  input_vars.Idq_req_UART[1] = 0;
	  }

	  //RCPWM input
	  if(input_vars.input_options & 0b0100){
		  if(input_vars.pulse_recieved){
			  if((input_vars.IC_duration > input_vars.IC_duration_MIN) && (input_vars.IC_duration < input_vars.IC_duration_MAX)){
				  if(input_vars.IC_pulse>(input_vars.IC_pulse_MID + input_vars.IC_pulse_DEADZONE)){
					  input_vars.Idq_req_RCPWM[0] = 0;
					  input_vars.Idq_req_RCPWM[1] = (float)(input_vars.IC_pulse - (input_vars.IC_pulse_MID + input_vars.IC_pulse_DEADZONE))*input_vars.RCPWM_gain[0][1];
				  }
				  else if(input_vars.IC_pulse<(input_vars.IC_pulse_MID - input_vars.IC_pulse_DEADZONE)){
					  input_vars.Idq_req_RCPWM[0] = 0;
					  input_vars.Idq_req_RCPWM[1] = ((float)input_vars.IC_pulse - (float)(input_vars.IC_pulse_MID - input_vars.IC_pulse_DEADZONE))*input_vars.RCPWM_gain[0][1];
				  }
				  else{
					  input_vars.Idq_req_RCPWM[0] = 0;
					  input_vars.Idq_req_RCPWM[1] = 0;
				  }
			  }	else {//The duration of the IC was wrong; trap it and write no current request
				  //Todo maybe want to implement a timeout on this, allowing spurious pulses to not wiggle the current?
				  input_vars.Idq_req_RCPWM[0] = 0;
				  input_vars.Idq_req_RCPWM[1] = 0;
			  }
		  }
		  else {//No pulse received flag
			  input_vars.Idq_req_RCPWM[0] = 0;
			  input_vars.Idq_req_RCPWM[1] = 0;
		  }
	  }

	  //ADC1 input
	  if(input_vars.input_options & 0b0010){
		  if(measurement_buffers.RawADC[1][3]>input_vars.adc1_MIN){
			  input_vars.Idq_req_ADC1[0] = 0;
			  input_vars.Idq_req_ADC1[1] = ((float)measurement_buffers.RawADC[1][3]-(float)input_vars.adc1_MIN)*input_vars.adc1_gain[1]*input_vars.ADC1_polarity;
		  }
		  else{
			  input_vars.Idq_req_ADC1[0] = 0;
			  input_vars.Idq_req_ADC1[1] = 0;
		  }
	  }
	  if(input_vars.input_options & 0b0001){
		  //ADC2 input
		  //placeholder
	  }

foc_vars.Idq_req[1] = input_vars.Idq_req_UART[1] + input_vars.Idq_req_RCPWM[1] + input_vars.Idq_req_ADC1[1] + input_vars.Idq_req_ADC2[1];

///////////// Clamp the overall request
if(foc_vars.Idq_req[0]>input_vars.max_request_Idq[0]){foc_vars.Idq_req[0] = input_vars.max_request_Idq[0];}
if(foc_vars.Idq_req[0]<input_vars.min_request_Idq[0]){foc_vars.Idq_req[0] = input_vars.min_request_Idq[0];}
if(foc_vars.Idq_req[1]>input_vars.max_request_Idq[1]){foc_vars.Idq_req[1] = input_vars.max_request_Idq[1];}
if(foc_vars.Idq_req[1]<input_vars.min_request_Idq[1]){foc_vars.Idq_req[1] = input_vars.min_request_Idq[1];}



////// Adjust the SVPWM gains to account for the change in battery voltage etc
    calculateVoltageGain();

////// Calculate the current power
    foc_vars.currentPower[0] = 1.5f*(foc_vars.Vdq[0]*foc_vars.Idq_smoothed[0]);
    foc_vars.currentPower[1] = 1.5f*(foc_vars.Vdq[1]*foc_vars.Idq_smoothed[1]);

    foc_vars.Ibus = (foc_vars.currentPower[0] + foc_vars.currentPower[1]) /measurement_buffers.ConvertedADC[0][1];

//Run MTPA (Field weakening seems to have to go in  the fast loop to be stable)
#ifdef USE_MTPA

    if(Lqd_diff>0){
    	foc_vars.id_mtpa = motor_profile->flux_linkage/(4.0f*Lqd_diff) - sqrtf((motor_profile->flux_linkage*motor_profile->flux_linkage/(16.0f*Lqd_diff*Lqd_diff))+foc_vars.Idq_req[1]*foc_vars.Idq_req[1]*0.5f);
    	if(foc_vars.Idq_req[1]>foc_vars.id_mtpa){
    	foc_vars.iq_mtpa = sqrtf(foc_vars.Idq_req[1]*foc_vars.Idq_req[1]-foc_vars.id_mtpa*foc_vars.id_mtpa);
    	}
    	else{
    		foc_vars.iq_mtpa = 0;
    	}
    foc_vars.Idq_req[0] = foc_vars.id_mtpa;
    if(foc_vars.Idq_req[1]>0.0f){
    foc_vars.Idq_req[1] = foc_vars.iq_mtpa;}
    else{
    	foc_vars.Idq_req[1] = -foc_vars.iq_mtpa;}
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
    	if(0){generateBreak(); //ToDo Currently not loading the profile so commented out - no temp safety!
    	MotorState = MOTOR_STATE_ERROR;
    	MotorError = MOTOR_ERROR_OVER_LIMIT_TEMP;
    	}
    }
/////// Clamp the max power taken from the battery
    foc_vars.reqPower = 1.5f*fabs(foc_vars.Vdq_smoothed[1] * foc_vars.Idq_req[1]);
    if (foc_vars.reqPower > motor_profile->Pmax) {
    	if(foc_vars.Idq_req[1] > 0.0f){
    		foc_vars.Idq_req[1] = motor_profile->Pmax / (fabs(foc_vars.Vdq_smoothed[1])*1.5f);
    	}else{
    		foc_vars.Idq_req[1] = -motor_profile->Pmax / (fabs(foc_vars.Vdq_smoothed[1])*1.5f);
    	}
    }

////// Unpuc the observer kludge
// The observer gets into a bit of a state if it gets close to
// flux linked = 0 for both accumuators, the angle rapidly changes
// as it oscillates around zero. Solution... just kludge it back out.
// This only happens at stationary when it is useless anyway.
    if ((flux_linked_alpha * flux_linked_alpha + flux_linked_beta * flux_linked_beta) <
        0.25f * motor_profile->flux_linkage * motor_profile->flux_linkage) {
      flux_linked_alpha = 0.5f * motor_profile->flux_linkage;
      flux_linked_beta = 0.5f * motor_profile->flux_linkage;
    }

    //////Set tracking
static int was_last_tracking;
if(!((MotorState==MOTOR_STATE_MEASURING)||(MotorState==MOTOR_STATE_DETECTING)||(MotorState==MOTOR_STATE_GET_KV))){
if(fabs(foc_vars.Idq_req[1])>0.1f){
	if(MotorState != MOTOR_STATE_ERROR){
#ifdef HAS_PHASE_SENSORS //We can go straight to RUN if we have been tracking with phase sensors
	MotorState = MOTOR_STATE_RUN;
#endif
if(MotorState==MOTOR_STATE_IDLE){
#ifdef USE_DEADSHORT
	MotorState = MOTOR_STATE_RECOVERING;

#endif
		}
	}
}else{
#ifdef HAS_PHASE_SENSORS
	MotorState = MOTOR_STATE_TRACKING;
#else
//	if(MotorState != MOTOR_STATE_ERROR){
	MotorState = MOTOR_STATE_IDLE;
//	}
#endif
was_last_tracking = 1;
}
}
//foc_vars.Idq_req[0] = 10; //for aligning encoder
/////////////Set and reset the HFI////////////////////////
#ifdef USE_HFI
    if((foc_vars.Vdq[1] > 2.0f)||(foc_vars.Vdq[1] < -2.0f)){
    	foc_vars.inject = 0;
    } else if((foc_vars.Vdq[1] < 1.0f)&&(foc_vars.Vdq[1] > -1.0f)){
    	foc_vars.inject = 1;
  	  foc_vars.Vd_injectionV = HFI_VOLTAGE;
  	  foc_vars.Vq_injectionV = 0.0f;
    }

    if(foc_vars.inject==1){
    	static int HFI_countdown;
    	static int no_q;
    	if(was_last_tracking==1){
    		HFI_countdown = 4; //resolve the ambiguity immediately
    		foc_vars.Idq_req[1] = 0.0f;
    		no_q=1;
    		was_last_tracking = 0;
    	}
		if(HFI_countdown==3){
			foc_vars.Idq_req[0] = 50.0f;
		}else if(HFI_countdown==2){
			foc_vars.Ldq_now_dboost[0] = foc_vars.IIR[0]; //Find the effect of d-axis current
			foc_vars.Idq_req[0] = 1.0f;
		}else if(HFI_countdown == 1){
			foc_vars.Idq_req[0] = -50.0f;
		}else if(HFI_countdown == 0){
			foc_vars.Ldq_now[0] = foc_vars.IIR[0];//foc_vars.Vd_injectionV;
			foc_vars.Idq_req[0] = 1.0f;
		if(foc_vars.Ldq_now[0]>foc_vars.Ldq_now_dboost[0]){foc_vars.FOCAngle+=32768;}
		HFI_countdown = 200;
		no_q = 0;
		}else{
			foc_vars.Idq_req[0] = 0.0f;
		}
		HFI_countdown--;
	    if(no_q){foc_vars.Idq_req[1]=0.0f;}
    }
#endif

    //Speed tracker
    if(0){//foc_vars.increment_count>0){
    foc_vars.eHz = ((foc_vars.angle_error*foc_vars.pwm_frequency)/(65536.0f*foc_vars.increment_count));
    foc_vars.increment_count = 0;
    foc_vars.angle_error = 0;
    }
  }
  }


  void MESCTrack() {
    // here we are going to do the clark and park transform of the voltages to
    // get the VaVb and VdVq These can be handed later to the observers and used
    // to set the integral terms

    // Clark transform
    foc_vars.Vab[0] =
        0.666f * (measurement_buffers.ConvertedADC[0][2] -
                  0.5f * ((measurement_buffers.ConvertedADC[1][1] - 0.3f) +
                          (measurement_buffers.ConvertedADC[1][2])));
    foc_vars.Vab[1] =
        0.666f *
        (sqrt3_on_2 * ((measurement_buffers.ConvertedADC[1][1]) -
                       (measurement_buffers.ConvertedADC[1][2] + 0.6f)));

    sin_cos_fast(foc_vars.FOCAngle, &foc_vars.sincosangle.sin, &foc_vars.sincosangle.cos);

    // Park transform

    foc_vars.Vdq[0] = foc_vars.sincosangle.cos * foc_vars.Vab[0] +
                      foc_vars.sincosangle.sin * foc_vars.Vab[1];
    foc_vars.Vdq[1] = foc_vars.sincosangle.cos * foc_vars.Vab[1] -
                      foc_vars.sincosangle.sin * foc_vars.Vab[0];
    foc_vars.Idq_int_err[1] = foc_vars.Vdq[1];
  }


  float IacalcDS, IbcalcDS, VacalcDS, VbcalcDS, VdcalcDS, VqcalcDS, FLaDS, FLbDS, FLaDSErr, FLbDSErr;
  uint16_t angleDS, angleErrorDSENC, angleErrorPhaseSENC, angleErrorPhaseDS, countdown_cycles;

  void deadshort(){
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

	  		if(countdown == 1||(((foc_vars.Iab[0]*foc_vars.Iab[0]+foc_vars.Iab[1]*foc_vars.Iab[1])>DEADSHORT_CURRENT*DEADSHORT_CURRENT)&&countdown<9))
	  				{
	  					//Need to collect the ADC currents here
	  					generateBreak();
	  					//Calculate the voltages in the alpha beta phase...
	  					IacalcDS = foc_vars.Iab[0];
	  					IbcalcDS = foc_vars.Iab[1];
	  					VacalcDS = -motor_profile->L_D*foc_vars.Iab[0]/((9.0f-(float)countdown)*foc_vars.pwm_period);
	  					VbcalcDS = -motor_profile->L_D*foc_vars.Iab[1]/((9.0f-(float)countdown)*foc_vars.pwm_period);
	  					//Calculate the phase angle
	  					//TEST LINE angleDS = (uint16_t)(32768.0f + 10430.0f * fast_atan2(VbcalcDS, VacalcDS)) - 32768;// +16384;

	  					 angleDS = (uint16_t)(32768.0f + 10430.0f * fast_atan2(VbcalcDS, VacalcDS)) - 32768 -16384;
	  					//Shifting by 1/4 erev does not work for going backwards. Need to rethink.
	  					//Problem is, depending on motor direction, the sign of the voltage generated swaps for the same rotor position.
	  					//The atan2(flux linkages) is stable under this regime, but the same for voltage is not.
	  					foc_vars.FOCAngle = angleDS;//foc_vars.enc_angle;//
	  					sin_cos_fast(foc_vars.FOCAngle, &foc_vars.sincosangle.sin, &foc_vars.sincosangle.cos);

	  					//Park transform it to get VdVq
	  					VdcalcDS = foc_vars.sincosangle.cos * VacalcDS +
	  				                      foc_vars.sincosangle.sin * VbcalcDS;
	  					VqcalcDS = foc_vars.sincosangle.cos * VbcalcDS -
	  				                      foc_vars.sincosangle.sin * VacalcDS;
	  					//Preloading the observer
	  					FLaDS = motor_profile->flux_linkage*foc_vars.sincosangle.cos;
	  					FLbDS = motor_profile->flux_linkage*foc_vars.sincosangle.sin;
	  		//Angle Errors for debugging
	  					angleErrorDSENC = angleDS-foc_vars.enc_angle;
	  		//			angleErrorPhaseSENC = foc_vars.FOCAngle-foc_vars.enc_angle;
	  		//			angleErrorPhaseDS = foc_vars.FOCAngle - angleDS;
	  		//Variables for monitoring and debugging to see if the preload will work
	  		//			FLaDSErr = 1000.0f*(FLaDS-flux_linked_alpha);
	  		//			FLbDSErr = 1000.0f*(FLbDS-flux_linked_beta);

	  		//Do actual preloading
	  					flux_linked_alpha = FLaDS;
	  					flux_linked_beta = FLbDS;
	  					Ia_last = 0.0f;
	  					Ib_last = 0.0f;
	  					foc_vars.Idq_int_err[0] = VdcalcDS;
	  					foc_vars.Idq_int_err[1] = VqcalcDS;
	  		//Next PWM cycle it  will jump to running state,
	  					MESCFOC();
	  					countdown_cycles = 9-countdown;
	  					countdown = 1;



	  		}
	  		if(countdown > 10){
	  			generateBreak();
	  			htim1.Instance->CCR1 = 50;
	  			htim1.Instance->CCR2 = 50;
	  			htim1.Instance->CCR3 = 50;
	  			//Preload the timer at mid
	  		}
	  		if(countdown <= 10 && countdown>1 ){
	  			htim1.Instance->CCR1 = 50;
	  			htim1.Instance->CCR2 = 50;
	  			htim1.Instance->CCR3 = 50;
	  			generateEnable();
	  		}
	  		if(countdown == 1 ){
					countdown = 15; //We need at least a few cycles for the current to relax
									//to zero in case of rapid switching between states
  					MotorState = MOTOR_STATE_RUN;

	  		}
	  		countdown--;
  }

  uint16_t ang_reg_v = 0x8021, data_v;
  float ang_v;
  void tle5012(void)
  {
#ifdef USE_ENCODER
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);


      HAL_SPI_Transmit(&hspi3, (uint8_t *)(&ang_reg_v), 1, 0xff);
      HAL_SPI_Receive(&hspi3, (uint8_t *)(&data_v), 1, 0xff);

      data_v = data_v & 0x7fff;
      foc_vars.enc_angle = POLE_PAIRS*((data_v *2)%POLE_ANGLE)-foc_vars.enc_offset;
      ang_v = data_v / (0x7fff / 360.0);
  //HAL_Delay(0);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

      //USARTx_Printf("angle: %d.%03d\n", (int)ang_v, (int)((ang_v * 1000) - (int)(ang_v)*1000));
  //    HAL_Delay(500);
#endif
  }
  // clang-format on
