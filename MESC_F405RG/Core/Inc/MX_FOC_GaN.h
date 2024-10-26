/*
 * mxlemming_FOC_GaN.h
 *
 *  Created on: Dec 16, 2022
 *      Author: HPEnvy
 */

#ifndef INC_MX_FOC_GAN_H_
#define INC_MX_FOC_GAN_H_

#define MCMASTER_70KV_8080

#define PWM_FREQUENCY 25000 //This is half the VESC zero vector frequency; i.e. 20k is equivalent to VESC 40k
#define CUSTOM_DEADTIME 50 //ns

#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 45.0f
#define ABS_MAX_BUS_VOLTAGE 45.0f
#define ABS_MIN_BUS_VOLTAGE 38.0f
//#define R_SHUNT 0.00333f //Not really true, the GaN board has hall sensors on phase with a 1mohm in paralell with 1.2mohm.
//#define OPGAIN 10.0f
#define R_SHUNT 1.0f
#define OPGAIN 0.066f*(1.0f/(1.0f+1.2f)) //66mV/A* ratio of current

#define R_VBUS_BOTTOM 3300.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 100000.0f



#define MAX_ID_REQUEST 2.0f
#define MAX_IQ_REQUEST 30.0f


//#define USE_FIELD_WEAKENINGV2

#define GET_THROTTLE_INPUT  _motor->Raw.ADC_in_ext1 = hadc1.Instance->JDR3;  // Throttle for IMS board


//#define USE_LR_OBSERVER

/////////////////////Related to ANGLE ESTIMATION////////////////////////////////////////
#define INTERPOLATE_V7_ANGLE
#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_SENSORLESS
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_HALL
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_OPENLOOP
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_ENCODER
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_HFI

#define USE_HFI
#define HFI_VOLTAGE 4.0f
#define HFI_TEST_CURRENT 0.0f
#define HFI_THRESHOLD 2.5f
#define HFI45
#define DEFAULT_HFI_TYPE HFI_TYPE_NONE
//#define DEFAULT_HFI_TYPE HFI_TYPE_45
//#define DEFAULT_HFI_TYPE HFI_TYPE_D
//#define DEFAULT_HFI_TYPE HFI_TYPE_SPECIAL

//#define USE_HALL_START
#define HALL_VOLTAGE_THRESHOLD 1.5f

//#define USE_SPI_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS 10
#define ENCODER_E_OFFSET 25000
#define POLE_ANGLE (65536/POLE_PAIRS)

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

#define FASTLED GPIOB
#define FASTLEDIO GPIO_PIN_5
#define FASTLEDIONO 5
#define SLOWLED GPIOB
#define SLOWLEDIO GPIO_PIN_7
#define SLOWLEDIONO 7

#endif /* INC_MX_FOC_GAN_H_ */
