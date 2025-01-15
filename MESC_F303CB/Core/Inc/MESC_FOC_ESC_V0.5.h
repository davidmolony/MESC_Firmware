/*
 * MP2_V0_1.h
 *
 *  Created on: Dec 16, 2022
 *      Author: HPEnvy
 */
 
#ifndef INC_MESC_FOC_ESC_V0_5_H_
#define INC_MESC_FOC_ESC_V0_5_H_
//Pick a motor for default
#define G30//VOILAMART1500W
#define PWM_FREQUENCY 20000 //This is half the VESC zero vector frequency; i.e. 20k is equivalent to VESC 40k
//#define CUSTOM_DEADTIME 600 //ns

#define SHUNT_POLARITY -1.0

#define ABS_MAX_PHASE_CURRENT 100.0f
#define ABS_MAX_BUS_VOLTAGE 40.0f
#define ABS_MIN_BUS_VOLTAGE 24.0f
#define R_SHUNT 0.0005f
//ToDo need to define using a discrete opamp with resistors to set gain vs using one with a specified gain
#define R_SHUNT_PULLUP 4700.0f //For discrete opamps
#define R_SHUNT_SERIES_RESISTANCE 150.0f //For discrete opamps
#define R_VBUS_BOTTOM 1500.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 82000.0f
#define OPGAIN 16.0f
#define USE_INTERNAL_OPAMPS
#define ADC_OFFSET_DEFAULT 1870.0f;

#define MAX_ID_REQUEST 10.0f
#define MAX_IQ_REQUEST 30.0f

#define SEVEN_SECTOR		//Normal SVPWM implemented as midpoint clamp. If not defined, you will get 5 sector, bottom clamp
//#define DEADTIME_COMP
#define DEADTIME_COMP_V 5 	//Arbitrary value for now, needs parametising.
							//Basically this is half the time between MOSoff and MOSon
							//and needs determining experimentally, either with openloop
							//sin wave drawing or by finding the zero current switching "power knee point"
							

//Inputs
#define GET_THROTTLE_INPUT _motor->Raw.ADC_in_ext1 = hadc2.Instance->JDR4;  // Throttle for MP2 with F405 pill

//Use the Ebike Profile tool
//#define USE_PROFILE

//#define USE_FIELD_WEAKENING
#define USE_FIELD_WEAKENINGV2
#define FIELD_WEAKENING_CURRENT 20.0f
#define FIELD_WEAKENING_THRESHOLD 0.8f

/////////////////////Related to ANGLE ESTIMATION////////////////////////////////////////
#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_SENSORLESS
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_HALL
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_OPENLOOP
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_ENCODER
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_HFI

//#define INTERPOLATE_V7_ANGLE



#define USE_HFI
#define HFI_VOLTAGE 4.0f
#define HFI_TEST_CURRENT 0.0f
#define HFI_THRESHOLD 2.5f
#define DEFAULT_HFI_TYPE HFI_TYPE_NONE
//#define DEFAULT_HFI_TYPE HFI_TYPE_45
//#define DEFAULT_HFI_TYPE HFI_TYPE_D
//#define DEFAULT_HFI_TYPE HFI_TYPE_SPECIAL
#define MAX_MODULATION 0.5f

#define USE_HALL_START
#define HALL_VOLTAGE_THRESHOLD 1.5f
#define MIN_HALL_FLUX_VOLTS 5.0f


//#define USE_SPI_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS 10
#define ENCODER_E_OFFSET 25000
#define POLE_ANGLE (65536/POLE_PAIRS)

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

#endif /* INC_MESC_FOC_ESC_V0_5_H_ */
