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

#include <stdbool.h>
#include "stm32fxxx_hal.h"
#include "MESCmotor_state.h"
#include "MESCtemp.h"

//#include "MESCposition.h"
#define LOGGING

#define FOC_PERIODS                (1)

//Default options which can be overwritten by user
#ifndef PWM_FREQUENCY
#define PWM_FREQUENCY 20000 //This is half the VESC zero vector frequency; i.e. 20k is equivalent to VESC 40k
#endif

#ifndef SLOW_LOOP_FREQUENCY
#define SLOW_LOOP_FREQUENCY 100 //Frequency of the slow loop (MIN: 16Hz!)
#endif
#ifndef SLOWTIM_SCALER
#define SLOWTIM_SCALER 1 //There is an annoying /2 on the htim2 and other random timers that is present in the F405 but not the F401 and some others. Unclear where to get this from HAL library.
#endif

#ifndef DEADTIME_COMP_V
#define DEADTIME_COMP_V 0 	//Arbitrary value for starting, needs determining through TEST_TYP_DEAD_TIME_IDENT.
#endif						//Basically this is half the time between MOSoff and MOSon
							//and needs dtermining experimentally, either with openloop
							//sin wave drawing or by finding the zero current switching "power knee point"
							//Not defining this uses 5 sector and overmodulation compensation
							//5 sector is harder on the low side FETs (for now)but offers equal performance at low speed, better at high speed.
#ifndef OVERMOD_DT_COMP_THRESHOLD
#define OVERMOD_DT_COMP_THRESHOLD 100	//Prototype concept that allows 100% (possibly greater) modulation by
										//skipping turn off when the modulation is close to VBus, then compensating next cycle.
										//Only works with 5 sector (bottom clamp) - comment out #define SEVEN_SECTOR
#endif

#ifndef MAX_MODULATION
#define MAX_MODULATION 0.95f //default is 0.95f, can allow higher or lower. up to
							//1.1 stable with 5 sector switching,
							//1.05 is advised as max for low side shunts
#endif

#ifndef MIN_HALL_FLUX_VOLTS
#define MIN_HALL_FLUX_VOLTS 10.0f
#endif

#ifndef I_MEASURE
#define I_MEASURE 20.0f //Higher setpoint for resistance measurement
#endif
#ifndef I_MEASURE_CLOSEDLOOP
#define I_MEASURE_CLOSEDLOOP 8.5f 	//After spinning up openloop and getting an approximation,
									//this current is used to driver the motor and collect a refined flux linkage
#endif
#ifndef V_MEASURE
#define V_MEASURE 4.0f 	//Voltage setpoint for measuring inductance
#endif
#ifndef ERPM_MEASURE
#define ERPM_MEASURE 3000.0f//Speed to do the flux linkage measurement at
#endif

#ifndef MIN_IQ_REQUEST
#define MIN_IQ_REQUEST -0.1f
#endif

#ifndef DEFAULT_BATTERY_CURRENT
#define DEFAULT_BATTERY_CURRENT 10.0f
#endif

#ifndef DEADSHORT_CURRENT
#define DEADSHORT_CURRENT 30.0f
#endif
//HFI related
#ifndef HFI_VOLTAGE
#define HFI_VOLTAGE 2.0f
#endif

#ifndef HFI_TEST_CURRENT
#define HFI_TEST_CURRENT 10.0f
#endif

#ifndef HFI_THRESHOLD
#define HFI_THRESHOLD 3.0f
#endif

#ifndef DEFAULT_HFI_TYPE
#define DEFAULT_HFI_TYPE HFI_TYPE_NONE
#endif

#ifndef DEFAULT_STARTUP_SENSOR
#define DEFAULT_STARTUP_SENSOR STARTUP_SENSOR_OPENLOOP
#endif

#ifndef CURRENT_BANDWIDTH
#define CURRENT_BANDWIDTH 0.15f*PWM_FREQUENCY //Note, current bandwidth in rads-1, PWMfrequency in Hz.
#endif

#ifndef DEFAULT_SPEED_KP
#define DEFAULT_SPEED_KP 0.5f //Amps per eHz
#endif
#ifndef DEFAULT_SPEED_KI
#define DEFAULT_SPEED_KI 0.1f //Amps per eHz per slowloop period... ToDo make it per second. At 100Hz slowloop, 0.1f corresponds to a 10Hz integral.
#endif

#ifndef ADC_OFFSET_DEFAULT
#define ADC_OFFSET_DEFAULT 2048.0f
#endif

#ifndef HALL_IIR
#define HALL_IIR 0.05f
#endif

#define HALL_IIRN (1.0f-HALL_IIR)

//Position and speed estimator defaults
#ifndef PLL_KP
#define PLL_KP 0.5f
#endif
#ifndef PLL_KI
#define PLL_KI 0.02f
#endif

#ifndef POS_KP
#define POS_KP 0.0002f
#endif
#ifndef POS_KI
#define POS_KI 0.1f
#endif
#ifndef POS_KD
#define POS_KD 0.002f
#endif


#ifndef DEFAULT_CONTROL_MODE
#define DEFAULT_CONTROL_MODE MOTOR_CONTROL_MODE_TORQUE
#endif

#ifndef ABS_MIN_BUS_VOLTAGE
#define ABS_MIN_BUS_VOLTAGE 12.0f //We do not run below the typical gate driver safe working voltage.
#endif


#ifndef ADC1OOR
#define ADC1OOR 4095
#endif

#ifndef ADC2OOR
#define ADC2OOR 4095
#endif

#ifndef SAFE_START_DEFAULT
#define SAFE_START_DEFAULT 100
#endif

#ifndef DEFAULT_ENCODER_POLARITY
#define DEFAULT_ENCODER_POLARITY 0
#endif
#ifndef ENCODER_E_OFFSET
#define ENCODER_E_OFFSET 0
#endif


#define clamp(value, min, max) (min < max           \
  ? (value < min ? min : value > max ? max : value) \
  : (value < max ? max : value > min ? min : value))



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

	TEMP MOS_temp;

	int Motor_T;
	TEMP Motor_temp;

	int16_t ADC_in_ext1;
	int16_t ADC_in_ext2;
}MESC_raw_typedef;

//extern MESC_raw_typedef motor1;

typedef struct {
	float Iu;
	float Iv;
	float Iw;
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


typedef struct MOTORProfile
{
    float       Imax;         // Amp
    float 		IBatmax;	  // Battery Amps
    float       Vmax;         // Volt
    float       Pmax;         // Watt
    uint32_t    RPMmax;       // 1/minute
    uint8_t     pole_pairs;
    uint16_t 	pole_angle;
    uint8_t     direction;
    uint8_t     _[2];
    float       L_D;          // Henry
    float       L_Q;          // Henry
    float 		L_QD;		  // Henry
    float       R;            // Ohm
    float       flux_linkage; // Weber
    float       flux_linkage_min;
    float       flux_linkage_max;
    float       flux_linkage_gain;
    float       non_linear_centering_gain; //Weber/second
    float 		hall_flux[6][2]; //Weber
    uint16_t 	hall_table[6][4];  // Lookup table, populated by the getHallTable()
    uint16_t 	enc_counts;
} MOTORProfile;


typedef struct {
  int initing;  // Flag to say we are initialising

  uint16_t openloop_step;//The angle to increment by for openloop
  uint16_t FOCAngle;    // Angle generated in the hall sensor estimator
  uint32_t encoder_duration;
  uint32_t encoder_pulse;
  uint32_t encoder_OK;
  uint16_t enc_angle;

  uint16_t enc_period_count;//For PWM encoder interpolation
  uint16_t enc_ratio;//For ABI encoder PPR to uint16_t conversion
  uint16_t last_enc_period;
  uint16_t last_enc_angle;
  int16_t enc_pwm_step;

  uint16_t enc_offset;
  float encsin;
  float enccos;
  uint16_t encoder_polarity_invert;
  int enc_obs_angle;
  uint16_t parkangle;
  float park_current;
  float park_current_now;

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
  float maxIgamma;


  float inverterVoltage[3];
  MESCiq_s Idq_req;							//The input to the PI controller. Load this with the values you want.
  MESCiq_s Idq_prereq2;
  MESCiq_s Idq_prereq; 						//Before we set the input to the current PI controller, we want to run a series of calcs (collect variables,
										  	  //calculate MTPA... that needs to be done without it putting jitter onto the PI input.
  float T_rollback;							//Scale the input parameters by this amount when thermal throttling
  MESCiq_s currentPower;					//Power being consumed by the motor; this does not include steady state losses and losses to switching
  float currentPowerab;
  float Ibus;
  float reqPower;
  float speed_req;
  float speed_kp;
  float speed_ki;
  float speed_error_int;

  //Observer parameters
  float Ia_last;
  float Ib_last;
  float La_last;
  float Lb_last;
  float flux_a;
  float flux_b;
  float flux_observed;
  float ortega_gain;

//Hall start
  uint16_t hall_initialised;
  int 			hall_start_now;
  float 		hall_IIR; //decay constant for the hall start preload
  float 		hall_IIRN;
  float 		hall_transition_V; //transition voltage above which the hall sensors are not doing any preloading
  //Encoder start
  int enc_start_now;

  float pwm_period;
  float pwm_frequency;

  float Current_bandwidth;
  float Id_pgain;  // Current controller gains
  float Id_igain;
  float Iq_pgain;
  float Iq_igain;
  float Vab_to_PWM;
  uint16_t deadtime_comp;
  float Modulation_max;
  float Duty_scaler;
  float Voltage;
  float Vmag_max;
  float V_3Q_mag_max;
  float Vmag_max2;
  float Vd_max;
  float Vq_max;
  float Vdint_max;
  float Vqint_max;
  float PWMmid;
  uint32_t ADC_duty_threshold;
  // Field weakenning
  float FW_curr_max;
  float FW_threshold;
  float FW_multiplier;
  float FW_current;
  float FW_ehz_max;
  float FW_estep_max;


  uint16_t state[4];  // current state, last state, angle change occurred
  uint16_t hall_update;
  uint32_t IRQentry;
  uint32_t IRQexit;

  MESCiq_s didq;
  int was_last_tracking;
  uint32_t FLrun, VFLrun;
  float PLL_error;
  float PLL_int;
  float PLL_kp;
  float PLL_ki;
  uint32_t PLL_angle;
  float eHz;
  float mechRPM;
  float Ldq_now[2];
  float Ldq_now_dboost[2];
  int d_polarity; //With this, we can swap the PLL polarity and therefore make it track Q instead of D. This is useful for detection

  float IIR[2];
  uint32_t cycles_fastloop;
  uint32_t cycles_pwmloop;
} MESCfoc_s;

extern MESCfoc_s foc_vars;

enum MEAS_ENUM
{
	MEAS_STATE_IDLE = 0,
	MEAS_STATE_INIT,
	MEAS_STATE_ALIGN,
	MEAS_STATE_LOWER_SETPOINT,
	MEAS_STATE_UPPER_SETPOINT_STABILISATION,
	MEAS_STATE_UPPER_SETPOINT,
	MEAS_STATE_INIT_LD,
	MEAS_STATE_INIT_LQ,
	MEAS_STATE_COLLECT_LD,
	MEAS_STATE_COLLECT_LQ,
};

typedef struct {
	//Measure resistance
	float top_V;
	float bottom_V;
	float top_I;
	float bottom_I;
	float count_top;
	float count_topq;
	float count_bottom;
	float count_bottomq;

	float Vd_temp;
	float Vq_temp;
	float top_I_L;
	float bottom_I_L;
	float top_I_Lq;
	float bottom_I_Lq;
	int PWM_cycles;
	HFI_type_e previous_HFI_type;

	//getkV
	int angle_delta;
	float temp_flux;
	float temp_FLA;
	float temp_FLB;

	float hfi_voltage;

	float measure_current;
	float measure_voltage;
	float measure_closedloop_current;
	uint32_t state;
} MESCmeas_s;

typedef struct {
	float dir;
	int current_hall_state;
	uint16_t current_hall_angle;
	int last_hall_state;
	uint16_t last_hall_angle;
	float ticks_since_last_observer_change;
	float last_observer_period;
	float one_on_last_observer_period;
	float angular_velocity;
	float angle_step;

	int hall_error;
} MESChall_s;

typedef struct{
	uint16_t OL_periods;
	uint16_t OL_countdown;
	int  closed_loop;
	int sector;
	int direction;
	float PWM_period;

	float I_set;
	float I_meas;
	float V_meas;
	float V_meas_sect[6];
	float rising_int;
	float falling_int;
	float rising_int_st;
	float falling_int_st;
	float last_p_error;

	float I_error;
	float int_I_error;
	float I_pgain;
	float I_igain;

	float com_flux;
	float flux_integral;
	float last_flux_integral;

	float V_bldc;
	float V_bldc_to_PWM;
	uint16_t BLDC_PWM;

}MESCBLDC_s;


/////////////Position controller data
typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float error;
	uint32_t last_pll_pos;
	float d_pos;
	float p_error;
	float d_error;
	float int_error;
	uint32_t set_position;
	int32_t deadzone;
}MESCPos_s;

//Logging
#ifndef LOGLENGTH
#define LOGLENGTH 300
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
	uint16_t hallstate[LOGLENGTH];

	uint32_t current_sample;
	bool sample_now;
	bool sample_no_auto_send;
	bool print_samples_now;
	bool lognow;
} MESClogging_s;

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
	uint32_t adc1_OOR;
	float adc1_gain[2];

	uint32_t adc2_MIN;
	uint32_t adc2_MAX;
	uint32_t adc2_OOR;

	float adc2_gain[2];

	float ADC1_polarity;
	float ADC2_polarity;

	float UART_req;
	float UART_dreq;
	float RCPWM_req;
	float ADC1_req;
	float ADC2_req;
	float ADC12_diff_req;


	uint8_t remote_ADC_can_id;
	float remote_ADC1_req;
	float remote_ADC2_req;
	int32_t remote_ADC_timeout;


	uint16_t nKillswitch;
	uint16_t invert_killswitch;

	uint32_t input_options; //	0b...tuvwxyz where
							//	t is differential ADC,
							//	u is ADC1 remote,
							//	v is ADC2 remote
							//	w is UART,
							//	x is RCPWM,
							//	y is ADC1
							//	z is ADC2

	MESCiq_s max_request_Idq;
	MESCiq_s min_request_Idq;
} input_vars_t;

typedef struct {
  float dp_current_final[10];
} MESCtest_s;

typedef struct {
	  float Vd_obs_high;
	  float Vd_obs_low;
	  float R_observer;
	  float Vq_obs_high;
	  float Vq_obs_low;
	  float L_observer;
	  float Last_eHz;
	  float LR_collect_count;
	  float Vd_obs_high_filt;
	  float Vd_obs_low_filt;
	  float Vq_obs_high_filt;
	  float Vq_obs_low_filt;
	  int plusminus;
} MESClrobs_s;

typedef struct {
	HFI_type_e Type;
	uint16_t inject;
	uint16_t inject_high_low_now;
	float Vd_injectionV;
	float Vq_injectionV;
	float special_injectionVd;
	float special_injectionVq;
	float toggle_voltage;
	float toggle_eHz;
	float mod_didq;
	float Gain;
	float int_err;
	float accu;
	int32_t countdown;
	uint32_t count;
	uint32_t test_increment;
} MESChfi_s;

enum FIELD_WEAKENING
{
	FIELD_WEAKENING_OFF = 0,
	FIELD_WEAKENING_V1 = 1,
	FIELD_WEAKENING_V2 = 2
};
enum OBSERVER_TYPE
{
	NONE = 0,
	MXLEMMING_LAMBDA = 1,
	MXLEMMING = 2,
	ORTEGA_ORIGINAL = 3
};

enum SQRT_CIRC
{
	SQRT_CIRCLE_LIM_OFF = 0,
	SQRT_CIRCLE_LIM_ON = 1,
	SQRT_CIRCLE_LIM_VD = 2
};
enum PWM_TYPE
{
	PWM_SVPWM = 0,
	PWM_SIN = 1,
	PWM_BOTTOM_CLAMP = 2,
	PWM_SIN_BOTTOM = 3
};
enum APP_TYPE
{
	APP_NONE = 0,
	APP_VEHICLE = 1,
	APP_2,
	APP_3
};

enum MTPA_MODE
{
	MTPA_NONE = 0,
	MTPA_REQ = 1,
	MTPA_MAG = 2,
	MTPA_Q = 3
};
typedef struct {
	bool use_hall_start;
	bool use_lr_observer;
	uint8_t MTPA_mode;
	bool use_phase_balancing;
	bool has_motor_temp_sensor;
	uint8_t field_weakening;
	uint8_t sqrt_circle_lim;
	uint8_t observer_type;
	uint8_t pwm_type;
	uint8_t app_type;
} MESCoptionFlags_s;

///////////////////////////////////////////////////////////////////////////////////////////
////////////////////////Main typedef for starting a motor instance////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
typedef struct{
	TIM_HandleTypeDef *mtimer; //3 phase PWM timer
	TIM_HandleTypeDef *stimer; //Timer that services the slowloop
	TIM_HandleTypeDef *enctimer; //Timer devoted to taking incremental encoder inputs
//problematic if there is no SPI allocated//	SPI_HandleTypeDef *encspi; //The SPI we have configured to talk to the encoder for this motor instance
	motor_state_e MotorState;
	motor_sensor_mode_e MotorSensorMode;
	motor_startup_sensor_e SLStartupSensor;
	motor_control_mode_e ControlMode;
	motor_control_type_e MotorControlType;
	HighPhase_e HighPhase;
	MESChfi_s HFI;
	MESC_raw_typedef Raw;
	MESC_Converted_typedef Conv;
	MESC_offset_typedef offset;
	MESCfoc_s FOC;
	MESCPos_s pos;
	MESCBLDC_s BLDC;
	MOTORProfile m;
	MESCmeas_s meas;
	MESChall_s hall;
	int32_t safe_start[2];
	uint32_t key_bits; //When any of these are low, we keep the motor disabled
	MESClogging_s logging;
	MESCtest_s test_vals;
	input_vars_t input_vars;
	MESClrobs_s lrobs;
	MESCoptionFlags_s options;
	bool conf_is_valid;
}MESC_motor_typedef;

extern MESC_motor_typedef mtr[NUM_MOTORS];


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








enum RCPWMMode{
	THROTTLE_ONLY,
	THROTTLE_REVERSE,
	THROTTLE_NO_REVERSE
};





/* Function prototypes -----------------------------------------------*/

void MESCfoc_Init(MESC_motor_typedef *_motor);
void initialiseInverter(MESC_motor_typedef *_motor);


void MESC_ADC_IRQ_handler(MESC_motor_typedef *_motor);
							//Put this into the ADC interrupt
							//Alternatively, the PWM and ADC IRQ handlers can be
							//stacked in a single interrupt occurring once per period
							//but HFI will be lost
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
float fast_atan2(float y, float x);
void angleObserver(MESC_motor_typedef *_motor);
void OLGenerateAngle(MESC_motor_typedef *_motor);  // For open loop FOC startup, just use this to generate
                         // an angle and velocity ramp, then keep the phase
                         // currents at the requested value without really
                         // thinking about things like synchronising, phase
                         // etc...

void MESCFOC(MESC_motor_typedef *_motor);  // Field and quadrature current control (PI?)
                 // Inverse Clark and Park transforms



void calculateGains(MESC_motor_typedef *_motor);
void calculateVoltageGain(MESC_motor_typedef *_motor);
void calculateFlux(MESC_motor_typedef *_motor);

//void MESCmeasure_DoublePulseTest(MESC_motor_typedef *_motor);

void MESC_Slow_IRQ_handler(MESC_motor_typedef *_motor); 	//This loop should run off a slow timer e.g. timer 3,4... at 20-50Hz in reset mode
														//Default setup is to use a 50Hz RCPWM input, which if the RCPWM is not present will run at 20Hz
														//If entered from update (reset, CC1) no data available for the PWM in. If entered from CC2, new PWM data available
void slowLoop(MESC_motor_typedef *_motor);
void MESCTrack(MESC_motor_typedef *_motor);
void deadshort(MESC_motor_typedef *_motor);
void tle5012(MESC_motor_typedef *_motor);
void HallFluxMonitor(MESC_motor_typedef *_motor);
void getIncEncAngle(MESC_motor_typedef *_motor);
void logVars(MESC_motor_typedef *_motor);
void printSamples(UART_HandleTypeDef *uart, DMA_HandleTypeDef *dma);
void RunMTPA(MESC_motor_typedef *_motor);
void safeStart(MESC_motor_typedef *_motor);

void RunSpeedControl(MESC_motor_typedef *_motor);

void MESC_IC_Init(
#ifdef IC_TIMER
TIM_HandleTypeDef _IC_TIMER
#endif
);
void MESC_IC_IRQ_Handler(MESC_motor_typedef *_motor, uint32_t SR, uint32_t CCR1, uint32_t CCR2);

#endif
