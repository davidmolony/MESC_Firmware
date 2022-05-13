#define PWM_FREQUENCY 20000 //This is half the VESC zero vector frequency; i.e. 20k is equivalent to VESC 40k
#define HAS_PHASE_SENSORS

#define SHUNT_POLARITY -1.0

#define ABS_MAX_PHASE_CURRENT 130.0f
#define ABS_MAX_BUS_VOLTAGE 85.0f
#define ABS_MIN_BUS_VOLTAGE 12.0f
#define R_SHUNT 0.0005f
//ToDo need to define using a discrete opamp with resistors to set gain vs using one with a specified gain
#define R_SHUNT_PULLUP 4700.0f //For discrete opamps
#define R_SHUNT_SERIES_RESISTANCE 150.0f //For discrete opamps
#define R_VBUS_BOTTOM 1500.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 82000.0f
#define OPGAIN 16.0f


#define MAX_ID_REQUEST 100.0f
#define MAX_IQ_REQUEST 100.0f


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

#define DEFAULT_INPUT	0b0110 //0b...wxyz where w is UART, x is RCPWM, y is ADC1 z is ADC2

#define USE_PROFILE
