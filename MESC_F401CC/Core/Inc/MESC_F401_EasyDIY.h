#define PWM_FREQUENCY 20000 //This is half the VESC zero vector frequency; i.e. 20k is equivalent to VESC 40k
//#define HAS_PHASE_SENSORS //This is not actually true. Really needs to have phase sensors... Leaving this in because it enables tracking and PWM disabling for debug.
//#define USE_DEADSHORT //This can be used in place of the phase sensors for startup from running.
#define DEADSHORT_CURRENT 30.0f	//When recovering from tracking phase without phase sensors, the
							//deadshort function will short the phases
							//until the current exceeds this value. At this point, it calculates the Vd Vq and phase angle
							//Don't set too high, after 9PWM periods, it will run the calc and start the motor regardless.
							//This seems to work best with a higher current bandwidth (~10krads-1) and using the non-linear observer centering.
							//Broadly incompatible with the flux observer
							//Only works for forward direction presently
							//^^WIP, not completely stable yet

//#define MISSING_UCURRSENSOR //You can run two current sensors ONLY if they are phase sensors.
//#define MISSING_VCURRSENSOR //Running this with low side sensors may result in fire.
//#define MISSING_WCURRSENSOR //Also requires that the third ADC is spoofed in the getRawADC(void) function in MESChw_setup.c to avoid trips

#define SOFTWARE_ADC_REGULAR
#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 60.0f
#define ABS_MAX_BUS_VOLTAGE 32.0f
#define ABS_MIN_BUS_VOLTAGE 10.0f
#define R_SHUNT 0.001f
//ToDo need to define using a discrete opamp with resistors to set gain vs using one with a specified gain

#define R_VBUS_BOTTOM 33000.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 1000000.0f
#define OPGAIN 10.0f


#define MAX_ID_REQUEST 10.0f
#define MAX_IQ_REQUEST 50.0f

#define I_MEASURE 30.0f //Higher setpoint for resistance measurement
#define V_MEASURE 4.0f 	//Voltage setpoint for measuring inductance

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

#define DEFAULT_INPUT	0b1110 //0b...wxyz where w is UART, x is RCPWM, y is ADC1 z is ADC2

//////Motor parameters
#define DEFAULT_MOTOR_POWER 250.0f
#define DEFAULT_FLUX_LINKAGE 0.0038f //mWb
#define DEFAULT_MOTOR_Ld 0.000005f //Henries
#define DEFAULT_MOTOR_Lq 0.000005f//Henries
#define DEFAULT_MOTOR_R 0.005f //Ohms
//Use the Ebike Profile tool
//#define USE_PROFILE
//#define USE_FIELD_WEAKENING
//#define USE_FIELD_WEAKENINGV2

#define FIELD_WEAKENING_CURRENT 10.0f
#define FIELD_WEAKENING_THRESHOLD 0.8f
//#define USE_HFI
#define HFI_VOLTAGE 4.0f
#define HFI_TEST_CURRENT 20.0f

#ifdef USE_HFI
#define CURRENT_BANDWIDTH 1000.0f //HFI does not work if the current controller is strong enough to squash the HFI
#else
#define CURRENT_BANDWIDTH 5000.0f
#endif

//#define USE_SQRT_CIRCLE_LIM

//#define USE_MTPA

/////Related to observer
#define USE_FLUX_LINKAGE_OBSERVER //This tracks the flux linkage in real time,
#define MAX_FLUX_LINKAGE DEFAULT_FLUX_LINKAGE*2.0f //Sets the limits for tracking.
#define MIN_FLUX_LINKAGE DEFAULT_FLUX_LINKAGE*0.7f//Faster convergence with closer start points
#define FLUX_LINKAGE_GAIN 10.0f * sqrtf(DEFAULT_FLUX_LINKAGE)//*(DEFAULT_FLUX_LINKAGE*DEFAULT_FLUX_LINKAGE)*PWM_FREQUENCY

//#define USE_NONLINEAR_OBSERVER_CENTERING //This is not a preferred option, since it relies on gain tuning and instability,
										//which is precisely what the original observer intended to avoid.
										//Also, incompatible with flux linkage observer for now...
#define NON_LINEAR_CENTERING_GAIN 5000.0f
#define USE_CLAMPED_OBSERVER_CENTERING //Pick one of the two centering methods... preferably this one
