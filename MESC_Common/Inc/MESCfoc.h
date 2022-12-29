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
 *  Created on: 18 Jul 2020
 *      Author: David Molony
 */

#ifndef MESC_FOC_H
#define MESC_FOC_H

#include "stm32fxxx_hal.h"
#include "MESCmotor_state.h"


#define FOC_PERIODS                (1)

//Default options which can be overwritten by user
#ifndef PWM_FREQUENCY
#define PWM_FREQUENCY 20000 //This is half the VESC zero vector frequency; i.e. 20k is equivalent to VESC 40k
#endif

#ifndef DEADTIME_COMP_V
#define DEADTIME_COMP_V 0 	//Arbitrary value for starting, needs determining through TEST_TYP_DEAD_TIME_IDENT.
#endif						//Basically this is half the time between MOSoff and MOSon
							//and needs dtermining experimentally, either with openloop
							//sin wave drawing or by finding the zero current switching "power knee point"
							//Not defining this uses 5 sector and overmodulation compensation
							//5 sector is harder on the low side FETs (for now)but offers equal performance at low speed, better at high speed.
#ifndef OVERMOD_DT_COMP_THRESHOLD
#define OVERMOD_DT_COMP_THRESHOLD 80	//Prototype concept that allows 100% (possibly greater) modulation by
										//skipping turn off when the modulation is close to VBus, then compensating next cycle.
										//Only works with 5 sector (bottom clamp) - comment out #define SEVEN_SECTOR
#endif

#ifndef MAX_MODULATION
#define MAX_MODULATION 0.95f //default is 0.95f, can allow higher or lower. up to
							//1.1 stable with 5 sector switching,
							//1.05 is advised as max for low side shunts
#endif

#ifndef I_MEASURE
#define I_MEASURE 10.0f //Higher setpoint for resistance measurement
#endif
#ifndef IMEASURE_CLOSEDLOOP
#define IMEASURE_CLOSEDLOOP 4.5f 	//After spinning up openloop and getting an approximation,
									//this current is used to driver the motor and collect a refined flux linkage
#endif
#ifndef V_MEASURE
#define V_MEASURE 4.0f 	//Voltage setpoint for measuring inductance
#endif
#ifndef ERPM_MEASURE
#define ERPM_MEASURE 3000.0f//Speed to do the flux linkage measurement at
#endif

#ifndef DEADSHORT_CURRENT
#define DEADSHORT_CURRENT 30.0f
#endif
//HFI related
#ifndef HFI_VOLTAGE
#define HFI_VOLTAGE 4.0f
#endif

#ifndef HFI_TEST_CURRENT
#define HFI_TEST_CURRENT 10.0f
#endif

#ifndef HFI_THRESHOLD
#define HFI_THRESHOLD 3.0f
#endif

#ifdef USE_HFI
#define CURRENT_BANDWIDTH 10000.0f
#else
#ifndef CURRENT_BANDWIDTH
#define CURRENT_BANDWIDTH 10000.0f
#endif
#endif

#ifndef ADC_OFFSET_DEFAULT
#define ADC_OFFSET_DEFAULT 2048.0f
#endif

typedef struct {
	int Iu;
	int Iv;
	int Iw;

	int Vbus;

	int Vu;
	int Vv;
	int Vw;

	int MOSu_T;
	int MOSv_T;
	int MOSw_T;

	int Motor_T;

	int ADC_in_ext1;
	int ADC_in_ext2;
}MESC_raw_typedef;

//extern MESC_raw_typedef motor1;

typedef struct {
	int Iu;
	int Iv;
	int Iw;
}MESC_offset_typedef;

typedef struct {
	float Iu;
	float Iv;
	float Iw;

	float Vbus;

	float Vu;
	float Vv;
	float Vw;

	float MOSu_T;
	float MOSv_T;
	float MOSw_T;

	float Motor_T;
}MESC_Converted_typedef;

typedef struct {
	float sin;
	float cos;
}MESCsin_cos_s;

typedef struct {
  float d;
  float q;
} MESCiq_s;

typedef struct {
  float a;
  float b;
  float g;
} MESCiab_s;

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
  MESCiab_s Vab;							//Float vector containing the Alpha beta frame voltage
  MESCiab_s Iab;							// Float vector containing the Clark transformed current in Amps
  MESCiq_s Idq;      						// Float vector containing the Park
  	  	  	  	  	  	  	  	  	  	  	// transformed current in amps
  MESCiq_s Vdq;
  MESCiq_s Idq_smoothed;
  MESCiq_s Idq_int_err;
  float id_mtpa;
  float iq_mtpa;


  float inverterVoltage[3];
  MESCiq_s Idq_req;							//The input to the PI controller. Load this with the values you want.
  MESCiq_s currentPower;					//Power being consumed by the motor; this does not include steady state losses and losses to switching
  float currentPowerab;
  float Ibus;
  float reqPower;

  //Observer parameters
  float Ia_last;
  float Ib_last;
  float La_last;
  float Lb_last;

  uint16_t hall_table[6][4];
  	  	  	  	  	  	  // Lookup table, populated by the getHallTable()
                          // function and used in estimating the rotor position
                          // from hall sensors in HallAngleEstimator()
  float hall_flux[6][2];
  	  	  	  	  	  	  //Lookup table for alpha beta fluxes per hall state
  int hall_initialised;
  int hall_forwards_adjust;
  int hall_backwards_adjust;

  float pwm_period;
  float pwm_frequency;

  float Current_bandwidth;
  float Id_pgain;  // Current controller gains
  float Id_igain;
  float Iq_pgain;
  float Iq_igain;
  float Vdqres_to_Vdq;
  float Vab_to_PWM;
  float Vmag_max;
  float Vmag_max2;
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
  float FW_current;

  float flux_a;
  float flux_b;
  uint16_t state[4];  // current state, last state, angle change occurred
  uint16_t hall_update;
  uint32_t IRQentry;
  uint32_t IRQexit;

  //HFI
  uint16_t inject;
  uint16_t inject_high_low_now;
  float Vd_injectionV;
  float Vq_injectionV;
  float HFI_Threshold;
  uint32_t FLrun, VFLrun;
  float angle_error;
  float eHz;
  float Ldq_now[2];
  float Ldq_now_dboost[2];
  int d_polarity; //With this, we can swap the PLL polarity and therefore make it track Q instead of D. This is useful for detection

  float IIR[2];
} MESCfoc_s;

extern MESCfoc_s foc_vars;

///////////////////////////////////////////////////////////////////////////////////////////
////////////////////////Main typedef for starting a motor instance////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
typedef struct{
	TIM_HandleTypeDef *mtimer; //3 phase PWM timer
	TIM_HandleTypeDef *stimer; //Timer that services the slowloop
	motor_state_e MotorState;
	motor_sensor_mode_e MotorSensorMode;
	HFI_type_e HFIType;
	MESC_raw_typedef Raw;
	MESC_Converted_typedef Conv;
	MESC_offset_typedef offset;
	MESCfoc_s FOC;
}MESC_motor_typedef;

extern MESC_motor_typedef motor1;
extern MESC_motor_typedef motor2;


enum MESCADC
{
    ADCIU,
    ADCIV,
    ADCIW,
};

#define SVPWM_MULTIPLIER \
  1.1547f  // 1/cos30 which comes from the maximum between two 120 degree apart
          // sin waves being at the
#define Vd_MAX_PROPORTION 0.3f //These are only used when hard clamping limits are enabled, not when SQRT circle limitation used
#define Vq_MAX_PROPORTION 0.95f

enum FOCChannels
{
    FOC_CHANNEL_PHASE_I,
    FOC_CHANNEL_DC_V,
    FOC_CHANNEL_PHASE_V,

    FOC_CHANNELS
};





typedef struct {
  float dp_current_final[10];
} MESCtest_s;

extern MESCtest_s test_vals;


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

	MESCiq_s Idq_req_UART;
	MESCiq_s Idq_req_RCPWM;
	MESCiq_s Idq_req_ADC1;
	MESCiq_s Idq_req_ADC2;

	uint32_t input_options; //0b...wxyz where w is UART, x is RCPWM, y is ADC1 z is ADC2

	MESCiq_s max_request_Idq;
	MESCiq_s min_request_Idq;
} input_vars_t;

extern input_vars_t input_vars;

//Logging
#ifndef LOGLENGTH
#define LOGLENGTH 100
#endif
//We want to log primarily Ia Ib Ic, Vd,Vq, phase angle, which gives us a complete picture of the machine state
//4 bytes per variable*6 variables*1000 = 24000bytes. Lowest spec target is F303CB with 48kB SRAM, so this is OK
typedef struct {
	float Vbus[LOGLENGTH];
	float Iu[LOGLENGTH];
	float Iv[LOGLENGTH];
	float Iw[LOGLENGTH];
	float Vd[LOGLENGTH];
	float Vq[LOGLENGTH];
	uint16_t angle[LOGLENGTH];
	uint32_t current_sample;
} sampled_vars_t;

extern int print_samples_now;

extern sampled_vars_t sampled_vars;

/* Function prototypes -----------------------------------------------*/

void MESCInit(MESC_motor_typedef *_motor);
void InputInit();
void initialiseInverter(MESC_motor_typedef *_motor);

void MESC_PWM_IRQ_handler(MESC_motor_typedef *_motor);
							//Put this into the PWM interrupt,
							//(or less optimally) ADC conversion complete interrupt
							//If using ADC interrupt, may want to get ADC to convert on top and bottom of PWM
void fastLoop(MESC_motor_typedef *_motor);
void hyperLoop(MESC_motor_typedef *_motor);
void VICheck(MESC_motor_typedef *_motor);
void ADCConversion(MESC_motor_typedef *_motor);  // Roll this into the V_I_Check? less branching, can
                       // probably reduce no.ops and needs doing every cycle
                       // anyway...
// convert currents from uint_16 ADC readings into float A and uint_16 voltages
// into float volts Since the observer needs the Clark transformed current, do
// the Clark and Park transform now
void ADCPhaseConversion(MESC_motor_typedef *_motor);
void hallAngleEstimator();  // Going to attempt to make a similar hall angle
                            // estimator that rolls the hall state into the main
                            // function, and calls a vector table to find the
                            // angle from hall offsets.
void flux_observer(MESC_motor_typedef *_motor);
float fast_atan2(float y, float x);
void angleObserver(MESC_motor_typedef *_motor);
void OLGenerateAngle(MESC_motor_typedef *_motor);  // For open loop FOC startup, just use this to generate
                         // an angle and velocity ramp, then keep the phase
                         // currents at the requested value without really
                         // thinking about things like synchronising, phase
                         // etc...

void MESCFOC(MESC_motor_typedef *_motor);  // Field and quadrature current control (PI?)
                 // Inverse Clark and Park transforms

void writePWM(MESC_motor_typedef *_motor);  // Offset the PWM to voltage centred (0Vduty is 50% PWM) or
                  // subtract lowest phase to always clamp one phase at 0V or
                  // SVPWM
                  // write CCR registers

void generateBreak(MESC_motor_typedef *_motor);  // Software break that does not stop the PWM timer but
                       // disables the outputs, sum of phU,V,W_Break();
void generateEnable(MESC_motor_typedef *_motor); // Opposite of generateBreak


void measureResistance(MESC_motor_typedef *_motor);
void measureInductance(MESC_motor_typedef *_motor);
void getkV(MESC_motor_typedef *_motor);

void getHallTable(MESC_motor_typedef *_motor);
void phU_Break(MESC_motor_typedef *_motor);   // Turn all phase U FETs off, Tristate the ouput - For BLDC
                    // mode mainly, but also used for measuring
void phU_Enable(MESC_motor_typedef *_motor);  // Basically un-break phase U, opposite of above...
void phV_Break(MESC_motor_typedef *_motor);
void phV_Enable(MESC_motor_typedef *_motor);
void phW_Break(MESC_motor_typedef *_motor);
void phW_Enable(MESC_motor_typedef *_motor);

void calculateGains(MESC_motor_typedef *_motor);
void calculateVoltageGain(MESC_motor_typedef *_motor);

void doublePulseTest(MESC_motor_typedef *_motor);

void MESC_Slow_IRQ_handler(MESC_motor_typedef *_motor); 	//This loop should run off a slow timer e.g. timer 3,4... at 20-50Hz in reset mode
														//Default setup is to use a 50Hz RCPWM input, which if the RCPWM is not present will run at 20Hz
														//If entered from update (reset, CC1) no data available for the PWM in. If entered from CC2, new PWM data available
void slowLoop(MESC_motor_typedef *_motor);
void MESCTrack(MESC_motor_typedef *_motor);
void deadshort(MESC_motor_typedef *_motor);
void tle5012();
void getDeadtime(MESC_motor_typedef *_motor);
void LRObserver(MESC_motor_typedef *_motor);
void LRObserverCollect(MESC_motor_typedef *_motor);
void HallFluxMonitor(MESC_motor_typedef *_motor);
void logVars(MESC_motor_typedef *_motor);
void printSamples(UART_HandleTypeDef *uart, DMA_HandleTypeDef *dma);
void RunHFI(MESC_motor_typedef *_motor);
#endif
