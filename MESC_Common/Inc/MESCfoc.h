/*
 **
 ******************************************************************************
 * @file           : MESCfoc.h
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

 * MESCfoc.h
 *
 *  Created on: 18 Jul 2020
 *      Author: David Molony
 */

#ifndef MESC_FOC_H
#define MESC_FOC_H

#include "stm32fxxx_hal.h"

#define FOC_SECTORS_PER_REVOLUTION (6)
#define FOC_CONV_CHANNELS          (4)
#define FOC_TRANSFORMED_CHANNELS   (2)
#define FOC_NUM_ADC                (4)
#define FOC_PERIODS                (1)

enum MESCADC
{
    ADCIU,
    ADCIV,
    ADCIW,
};

#define MAX_MODULATION 0.95
#define SVPWM_MULTIPLIER \
  1.1547  // 1/cos30 which comes from the maximum between two 120 degree apart
          // sin waves being at the
#define Vd_MAX_PROPORTION 0.3
#define Vq_MAX_PROPORTION 0.95

enum FOCChannels
{
    FOC_CHANNEL_PHASE_I,
    FOC_CHANNEL_DC_V,
    FOC_CHANNEL_PHASE_V,

    FOC_CHANNELS
};

typedef struct {
	float sin;
	float cos;
}MESCsin_cos_s;

typedef struct {
  int initing;  // Flag to say we are initialising

  uint16_t openloop_step;//The angle to increment by for openloop
  uint16_t FOCAngle;    // Angle generated in the hall sensor estimator
  uint16_t enc_angle;
  uint16_t enc_offset;
  int enc_obs_angle;
  float FLAdiff;
  MESCsin_cos_s sincosangle;  // This variable carries the current sin and cosine of
                         	  // the angle being used for Park and Clark transforms,
                              // so they only need computing once per pwm cycle
  float Iab[FOC_TRANSFORMED_CHANNELS + 1];  // Float vector containing the Clark
                                            // transformed current in amps
  float Idq[FOC_TRANSFORMED_CHANNELS];      // Float vector containing the Park
                                            // transformed current in amps
  float Vab[FOC_TRANSFORMED_CHANNELS + 1];
  float Vdq[FOC_TRANSFORMED_CHANNELS];
  float Vdq_smoothed[FOC_TRANSFORMED_CHANNELS];
  float Idq_smoothed[FOC_TRANSFORMED_CHANNELS];
  float Idq_int_err[2];
  float id_mtpa;
  float iq_mtpa;


  float inverterVoltage[FOC_TRANSFORMED_CHANNELS + 1];
  float Idq_req[2];							//The input to the PI controller. Load this with the values you want.
  float currentPower[2];					//Power being consumed by the motor; this does not include steady state losses and losses to switching
  float Ibus;
  float reqPower;

  uint16_t hall_table[6]
                     [4];  // Lookup table, populated by the getHallTable()
                           // function and used in estimating the rotor position
                           // from hall sensors in HallAngleEstimator()
  int hall_forwards_adjust;
  int hall_backwards_adjust;

  float pwm_period;
  float pwm_frequency;

  float Id_pgain;  // Current controller gains
  float Id_igain;
  float Iq_pgain;
  float Iq_igain;
  float Vdqres_to_Vdq;
  float Vab_to_PWM;
  float Vd_max;
  float Vq_max;
  float Vdint_max;
  float Vqint_max;
  float PWMmid;
  uint32_t ADC_duty_threshold;
  // Field weakenning
  float field_weakening_curr_max;
  float field_weakening_threshold;
  float field_weakening_multiplier;
  int field_weakening_flag;

  float VBEMFintegral[2];
  uint16_t state[4];  // current state, last state, angle change occurred
  uint16_t hall_update;
  uint16_t BEMF_update;
  uint32_t IRQentry;
  uint32_t IRQexit;
  uint16_t inject;
  uint16_t inject_high_low_now;
  float Vd_injectionV;
  float Vq_injectionV;
  uint32_t FLrun, VFLrun;
  int angle_error;
  float Ldq_now[2];
  float Ldq_now_dboost[2];

  float IIR[2];
} MESCfoc_s;

extern MESCfoc_s foc_vars;

typedef struct {
  float dp_current_final[10];
} MESCtest_s;

extern MESCtest_s test_vals;

typedef struct {
  int32_t RawADC[FOC_NUM_ADC]
                [FOC_CONV_CHANNELS];  // ADC1 returns Ucurrent, DClink
                                      // voltage and U phase voltage
                                      //  ADC2 returns Vcurrent, V and Wphase
                                      //  voltages
                                      // ADC3 returns Wcurrent
  // We can use ints rather than uints, since this later helps the conversion of
  // values to float, and the sign bit remains untouched (0)
  int32_t ADCOffset[FOC_NUM_ADC];  // During detect phase, need to sense the
                                   // zero current offset
  float ConvertedADC[FOC_NUM_ADC]
                    [FOC_CONV_CHANNELS];  // We will fill this with currents
                                          // in A and voltages in Volts
  uint32_t adc1, adc2, adc3, adc4, adc5;

} foc_measurement_t;

extern foc_measurement_t measurement_buffers;  // fixme: floating function prototype

enum RCPWMMode{
	THROTTLE_ONLY,
	THROTTLE_REVERSE,
	THROTTLE_NO_REVERSE
};

typedef struct {

	///////////////////RCPWM//////////////////////
	uint32_t IC_duration; 	//Retrieve this from timer input capture CC1
	uint32_t IC_pulse; 		//Retrieve this from timer input capture CC2
	uint32_t pulse_recieved;

	uint32_t IC_duration_MAX;
	uint32_t IC_duration_MIN;
	uint32_t IC_pulse_MAX;
	uint32_t IC_pulse_MIN;
	uint32_t IC_pulse_MID;
	uint32_t IC_pulse_DEADZONE; //single sided; no response before MID +- this
	float RCPWM_gain[2][2];

	 uint32_t fCC1;
	 uint32_t fUPD;

/////////////////ADC///////////////
	uint32_t adc1_MIN; //Value below which response is zero
	uint32_t adc1_MAX; //Max throttle calculated at this point
	float adc1_gain[2];

	uint32_t adc2_MIN;
	uint32_t adc2_MAX;
	float adc2_gain[2];

	float ADC1_polarity;
	float ADC2_polarity;

	float Idq_req_UART[2];
	float Idq_req_RCPWM[2];
	float Idq_req_ADC1[2];
	float Idq_req_ADC2[2];

	uint32_t input_options; //0b...wxyz where w is UART, x is RCPWM, y is ADC1 z is ADC2

float max_request_Idq[2];
float  min_request_Idq[2];
} input_vars_t;

extern input_vars_t input_vars;

/* Function prototypes -----------------------------------------------*/

void MESCInit();
void InputInit();
void MESC_PWM_IRQ_handler(); //Put this into the PWM interrupt,
							//(or less optimally) ADC conversion complete interrupt
							//If using ADC interrupt, may want to get ADC to convert on top and bottom of PWM
void fastLoop();
void hyperLoop();
void VICheck();
void ADCConversion();  // Roll this into the V_I_Check? less branching, can
                       // probably reduce no.ops and needs doing every cycle
                       // anyway...
// convert currents from uint_16 ADC readings into float A and uint_16 voltages
// into float volts Since the observer needs the Clark transformed current, do
// the Clark and Park transform now
void hallAngleEstimator();  // Going to attempt to make a similar hall angle
                            // estimator that rolls the hall state into the main
                            // function, and calls a vector table to find the
                            // angle from hall offsets.
void flux_observer();
float fast_atan2(float y, float x);
void angleObserver();
void fluxIntegrator();
void OLGenerateAngle();  // For open loop FOC startup, just use this to generate
                         // an angle and velocity ramp, then keep the phase
                         // currents at the requested value without really
                         // thinking about things like synchronising, phase
                         // etc...

void observerTick();  // Call every time to allow the observer, whatever it is,
                      // to update itself and find motor position

void MESCFOC();  // Field and quadrature current control (PI?)
                 // Inverse Clark and Park transforms

void writePWM();  // Offset the PWM to voltage centred (0Vduty is 50% PWM) or
                  // subtract lowest phase to always clamp one phase at 0V or
                  // SVPWM
                  // write CCR registers

void generateBreak();  // Software break that does not stop the PWM timer but
                       // disables the outputs, sum of phU,V,W_Break();
void generateEnable(); // Opposite of generateBreak

int isMotorRunning();  // return motor state if state is one of the running
                       // states, if it's an idle, error or break state, disable
                       // all outputs and measure the phase voltages - if all
                       // the same, then it's stationary.
void measureResistance();
void measureInductance();
void getkV();

void getHallTable();
void phU_Break();   // Turn all phase U FETs off, Tristate the ouput - For BLDC
                    // mode mainly, but also used for measuring
void phU_Enable();  // Basically un-break phase U, opposite of above...
void phV_Break();
void phV_Enable();
void phW_Break();
void phW_Enable();

void calculateGains();
void calculateVoltageGain();

void doublePulseTest();

void MESC_Slow_IRQ_handler(TIM_HandleTypeDef *htim); 	//This loop should run off a slow timer e.g. timer 3,4... at 20-50Hz in reset mode
														//Default setup is to use a 50Hz RCPWM input, which if the RCPWM is not present will run at 20Hz
														//If entered from update (reset, CC1) no data available for the PWM in. If entered from CC2, new PWM data available
void slowLoop(TIM_HandleTypeDef *htim);
void MESCTrack();
void tle5012();

#endif
