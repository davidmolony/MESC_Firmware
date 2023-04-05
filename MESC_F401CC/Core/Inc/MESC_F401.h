/*
 * MESC_F401.h
 *
 *  Created on: Dec 17, 2022
 *      Author: D Molony
 */

#ifndef INC_MESC_F401_H_
#define INC_MESC_F401_H_
//First, include the header specific to your board, which includes hardware parameters like ABS MAX, shunts, potential divdiders
//Ensure only one board's header file is uncommented!
#include "MP2_V0_1.h"


#define SOFTWARE_ADC_REGULAR

#define HAS_PHASE_SENSORS //This refers to VOLTAGE sensing on phase, not current!

#define NUM_MOTORS 1

//#define MISSING_UCURRSENSOR //You can run two current sensors ONLY if they are phase sensors.
//#define MISSING_VCURRSENSOR //Running this with low side sensors may result in fire.
//#define MISSING_WCURRSENSOR //Also requires that the third ADC is spoofed in the getRawADC(void) function in MESChw_setup.c to avoid trips



////////////////////USER DEFINES//////////////////
	///////////////////RCPWM//////////////////////
#define IC_DURATION_MAX 25000
#define IC_DURATION_MIN 15000

#define IC_PULSE_MAX 2100
#define IC_PULSE_MIN 900
#define IC_PULSE_MID 1500

#define IC_PULSE_DEADZONE 100


	/////////////////ADC///////////////
#define  ADC1MIN 1200
#define  ADC1MAX 2700
#define  ADC2MIN 1200
#define  ADC2MAX 4095

#define ADC1_POLARITY 1.0f
#define ADC2_POLARITY -1.0f

#define DEFAULT_INPUT	0b1001 //0b...wxyz where w is UART, x is RCPWM, y is ADC2 z is ADC1


//Use the Ebike Profile tool
#define USE_PROFILE

#ifndef FIELD_WEAKENING_CURRENT
#define FIELD_WEAKENING_CURRENT 10.0f //This does not set whether FW is used, just the default current
#endif

#ifndef FIELD_WEAKENING_THRESHOLD
#define FIELD_WEAKENING_THRESHOLD 0.8f
#endif


/////////////////////Related to CIRCLE LIMITATION////////////////////////////////////////
//#define USE_SQRT_CIRCLE_LIM
#define USE_SQRT_CIRCLE_LIM_VD

//#define USE_MTPA

/////////////////////Related to ONLINE PARAMETER ESTIMATION//////////////////////////////
#ifndef LR_OBS_CURRENT
#define LR_OBS_CURRENT 0.1*MAX_IQ_REQUEST 	//Inject this much current into the d-axis at the slowloop frequency and observe the change in Vd and Vq
								//Needs to be a small current that does not have much effect on the running parameters.
#endif

/////////////////////Related to OBSERVER//////////////////////////////
#define USE_FLUX_LINKAGE_OBSERVER //This tracks the flux linkage in real time,
#define MAX_FLUX_LINKAGE DEFAULT_FLUX_LINKAGE*2.0f //Sets the limits for tracking.
#define MIN_FLUX_LINKAGE DEFAULT_FLUX_LINKAGE*0.7f//Faster convergence with closer start points
#define FLUX_LINKAGE_GAIN 10.0f * sqrtf(DEFAULT_FLUX_LINKAGE)//*(DEFAULT_FLUX_LINKAGE*DEFAULT_FLUX_LINKAGE)*PWM_FREQUENCY

//#define USE_NONLINEAR_OBSERVER_CENTERING //This is not a preferred option, since it relies on gain tuning and instability,
										//which is precisely what the original observer intended to avoid.
										//Also, incompatible with flux linkage observer for now...
#define NON_LINEAR_CENTERING_GAIN 5000.0f
#define USE_CLAMPED_OBSERVER_CENTERING //Pick one of the two centering methods... preferably this one

/////////////////////Related to COMMs INTERFACE//////////////////////////////
#define MESC_UART_USB 		MESC_USB
#define HW_UART huart3

/////////////////////Prototype stuff that does not really work nicely//////////////////////////////

//#define USE_DEADSHORT //This can be used in place of the phase sensors for startup from running.
#define DEADSHORT_CURRENT 30.0f	//When recovering from tracking phase without phase sensors, the
							//deadshort function will short the phases
							//until the current exceeds this value. At this point, it calculates the Vd Vq and phase angle
							//Don't set too high, after 9PWM periods, it will run the calc and start the motor regardless.
							//This seems to work best with a higher current bandwidth (~10krads-1) and using the non-linear observer centering.
							//Broadly incompatible with the flux observer
							//Only works for forward direction presently
							//^^WIP, not completely stable yet


#endif /* INC_MESC_F401_H_ */
