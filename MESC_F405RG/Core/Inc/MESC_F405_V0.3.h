#define PWM_FREQUENCY 25000 //This is half the VESC zero vector frequency; i.e. 20k is equivalent to VESC 40k
#define HAS_PHASE_SENSORS //This refers to VOLTAGE sensing on phase, not current!
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

#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.
#define DEADTIME_COMP_V 5 	//Arbitrary value for starting, needs determining through TEST_TYP_DEAD_TIME_IDENT.
							//Basically this is half the time between MOSoff and MOSon
							//and needs dtermining experimentally, either with openloop
							//sin wave drawing or by finding the zero current switching "power knee point"
//#define SEVEN_SECTOR
#define OVERMOD_DT_COMP_THRESHOLD 80	//Prototype concept that allows 100% (possibly greater) modulation by
										//skipping turn off when the modulation is close to VBus, then compensating next cycle.
										//Only works with 5 sector (bottom clamp) - comment out #define SEVEN_SECTOR
#define MAX_MODULATION 0.99f //default is 0.95f, can allow higher or lower.
#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 250.0f
#define ABS_MAX_BUS_VOLTAGE 45.0f
#define ABS_MIN_BUS_VOLTAGE 24.0f
#define R_SHUNT 0.000333f
//ToDo need to define using a discrete opamp with resistors to set gain vs using one with a specified gain

//#define R_VBUS_BOTTOM 1500.0f //Phase and Vbus voltage sensors
//#define R_VBUS_TOP 82000.0f
//#define R_VBUS_BOTTOM 3300.0f //Phase and Vbus voltage sensors
//#define R_VBUS_TOP 100000.0f
#define R_VBUS_BOTTOM 2700.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 100000.0f
#define OPGAIN 10.0f


#define MAX_ID_REQUEST 10.0f
#define MAX_IQ_REQUEST 200.0f

#define I_MEASURE 15.0f //Higher setpoint for resistance measurement
#define IMEASURE_CLOSEDLOOP 1.5f 	//After spinning up openloop and getting an approximation,
									//this current is used to driver the motor and collect a refined flux linkage
#define V_MEASURE 4.0f 	//Voltage setpoint for measuring inductance
#define ERPM_MEASURE 7000.0f//Speed to do the flux linkage measurement at

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
#define DEFAULT_FLUX_LINKAGE 0.00380f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000006f //Henries
#define DEFAULT_MOTOR_Lq 0.0000120f//Henries
#define DEFAULT_MOTOR_R 0.0105260f //Ohms
//Use the Ebike Profile tool
#define USE_PROFILE

//#define USE_FIELD_WEAKENING
#define USE_FIELD_WEAKENINGV2

#define FIELD_WEAKENING_CURRENT 10.0f
#define FIELD_WEAKENING_THRESHOLD 0.8f
//#define USE_HFI
#define HFI_VOLTAGE 4.0f
#define HFI_TEST_CURRENT 10.0f

#ifdef USE_HFI
#define CURRENT_BANDWIDTH 1000.0f //HFI does not work if the current controller is strong enough to squash the HFI
#else
#define CURRENT_BANDWIDTH 5000.0f
#endif

//#define DO_OPENLOOP //A fudge that can be used for openloop testing; disable HFI

//#define USE_SQRT_CIRCLE_LIM
#define USE_SQRT_CIRCLE_LIM_VD
//#define USE_LR_OBSERVER
#define LR_OBS_CURRENT 0.1*MAX_IQ_REQUEST 	//Inject this much current into the d-axis at the slowloop frequency and observe the change in Vd and Vq
								//Needs to be a small current that does not have much effect on the running parameters.
								
//#define USE_MTPA //Cannot currently use at the same time as field weakening...

//#define USE_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS 10
#define ENCODER_E_OFFSET 25000
#define POLE_ANGLE (65536/POLE_PAIRS)

/////Related to observer
//#define USE_SALIENT_OBSERVER
#define USE_FLUX_LINKAGE_OBSERVER //This tracks the flux linkage in real time,
#define MAX_FLUX_LINKAGE DEFAULT_FLUX_LINKAGE*2.0f //Sets the limits for tracking.
#define MIN_FLUX_LINKAGE DEFAULT_FLUX_LINKAGE*0.7f//Faster convergence with closer start points
#define FLUX_LINKAGE_GAIN 10.0f * sqrtf(DEFAULT_FLUX_LINKAGE)//*(DEFAULT_FLUX_LINKAGE*DEFAULT_FLUX_LINKAGE)*PWM_FREQUENCY

#define USE_HALL_START

//#define USE_NONLINEAR_OBSERVER_CENTERING //This is not a preferred option, since it relies on gain tuning and instability,
										//which is precisely what the original observer intended to avoid.
										//Also, incompatible with flux linkage observer for now...
#define NON_LINEAR_CENTERING_GAIN 5000.0f
#define USE_CLAMPED_OBSERVER_CENTERING //Pick one of the two centering methods... preferably this one

#define MESC_UART_USB 		MESC_USB



// Example motors
//  motor.motor_flux = 0.000092; //Propdrive 2826 1200kV
//  motor.motor_flux = 0.011f; //Red 70kV McMaster 8080 motor
//  motor.Lphase = 0.00010f;	//Red 70kV McMaster 8080 motor
//  motor.Rphase = 0.042f;
//  motor.motor_flux = 0.014f; //Alien 8080 50kV motor
//  motor.motor_flux = 0.007f; 	//AT12070 62kV
//  motor.Lphase = 0.000016f;		//AT12070 62kV
//  motor.Rphase = 0.012f;		//AT12070 62kV
//  motor.motor_flux = 0.0041f; //CA120 150kV
//  motor.Lphase = 0.000006f;//CA120 150kV
//  motor.Rphase = 0.006f;//CA120 150kV
