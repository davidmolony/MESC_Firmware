/*
 * MP2_V0_1.h
 *
 *  Created on: Dec 16, 2022
 *      Author: HPEnvy
 */

#ifndef INC_MP2_V0_1_H_
#define INC_MP2_V0_1_H_
//Pick a motor for default
#define MCMASTER_70KV_8080//QS165

#define PWM_FREQUENCY 25000

#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 100.0f
#define ABS_MAX_BUS_VOLTAGE 45.0f
#define ABS_MIN_BUS_VOLTAGE 38.0f
#define R_SHUNT 0.00033f
#define OPGAIN 10.5f

#define R_VBUS_BOTTOM 3300.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 100000.0f


#define MAX_ID_REQUEST 2.0f
#define MAX_IQ_REQUEST 30.0f

#define SEVEN_SECTOR		//Normal SVPWM implemented as midpoint clamp. If not defined, you will get 5 sector, bottom clamp
//#define DEADTIME_COMP		//This injects extra PWM duty onto theF timer which effectively removes the dead time.
#define DEADTIME_COMP_V 10

//Inputs
#define GET_THROTTLE_INPUT _motor->Raw.ADC_in_ext1 = ADC_buffer[3]  // Throttle

#define USE_FIELD_WEAKENINGV2

//#define USE_LR_OBSERVER

/////////////////////Related to ANGLE ESTIMATION////////////////////////////////////////
//#define INTERPOLATE_V7_ANGLE

#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_SENSORLESS

//#define USE_HFI
#define HFI_VOLTAGE 8.0f
#define HFI_TEST_CURRENT 0.0f
#define HFI_THRESHOLD 2.5f
#define HFI45
#define DEFAULT_HFI_TYPE HFI_TYPE_NONE
//#define DEFAULT_HFI_TYPE HFI_TYPE_45
//#define DEFAULT_HFI_TYPE HFI_TYPE_D
//#define DEFAULT_HFI_TYPE HFI_TYPE_SPECIAL

//#define USE_HALL_START
#define HALL_VOLTAGE_THRESHOLD 1.5f

//#define USE_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS 10
#define ENCODER_E_OFFSET 25000
#define POLE_ANGLE (65536/POLE_PAIRS)

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

//LEDs
#define SLOWLED GPIOC
#define SLOWLEDIO GPIO_PIN_13
#define SLOWLEDIONO 13

#endif /* INC_MP2_V0_1_H_ */
