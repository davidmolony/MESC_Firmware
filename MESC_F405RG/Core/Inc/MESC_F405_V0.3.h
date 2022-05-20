#define PWM_FREQUENCY 20000 //This is half the VESC zero vector frequency; i.e. 20k is equivalent to VESC 40k
#define HAS_PHASE_SENSORS

#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 100.0f
#define ABS_MAX_BUS_VOLTAGE 55.0f
#define ABS_MIN_BUS_VOLTAGE 24.0f
#define R_SHUNT 0.000333f
//ToDo need to define using a discrete opamp with resistors to set gain vs using one with a specified gain

#define R_VBUS_BOTTOM 1500.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 82000.0f
#define OPGAIN 10.0f


#define MAX_ID_REQUEST 100.0f
#define MAX_IQ_REQUEST 40.0f


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

#define DEFAULT_MOTOR_FLUX 0.0038f //mWb
#define DEFAULT_MOTOR_Ld 0.000005f //Henries
#define DEFAULT_MOTOR_Lq 0.000005f//Henries
#define DEFAULT_MOTOR_R 0.005f //Ohms
//Use the Ebike Profile tool
//#define USE_PROFILE

//#define USE_FIELD_WEAKENING
#define FIELD_WEAKENING_CURRENT 10.0f
#define FIELD_WEAKENING_THRESHOLD 0.8f
//#define USE_HFI
#define USE_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS 10
#define ENCODER_E_OFFSET 25000
#define POLE_ANGLE (65536/POLE_PAIRS)
