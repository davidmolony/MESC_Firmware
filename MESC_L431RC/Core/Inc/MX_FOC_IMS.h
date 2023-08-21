/*
 * MX_FOC_IMS.h
 *
 *  Created on: Dec 16, 2022
 *      Author: D Molony
 */

#ifndef INC_MX_FOC_IMS_H_
#define INC_MX_FOC_IMS_H_

//Pick a motor for default
#define QS165V2//MCMASTER_70KV_8080

#define PWM_FREQUENCY 25000
#define CUSTOM_DEADTIME 700 //ns

#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 100.0f
#define ABS_MAX_BUS_VOLTAGE 50.0f
#define ABS_MIN_BUS_VOLTAGE 38.0f
#define R_SHUNT 0.00025f
#define OPGAIN 20.0f*30.0f/(4.7f+4.7f+30.0f) //Control board v0.1 has a 4.7-30-4.7 divider onto the opamp input to enable some filtering

#define R_VBUS_BOTTOM 3300.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 100000.0f

#define DEFAULT_INPUT	0b1010 //0b...wxyz where w is UART, x is RCPWM, y is ADC2 z is ADC1

#define MAX_ID_REQUEST 2.0f
#define MAX_IQ_REQUEST 20.0f
#define MIN_IQ_REQUEST -5.0f

#define SEVEN_SECTOR		//Normal SVPWM implemented as midpoint clamp. If not defined, you will get 5 sector, bottom clamp
//#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.
#define DEADTIME_COMP_V 10

#define USE_FIELD_WEAKENINGV2
//#define USE_FIELD_WEAKENINGV3


#define GET_THROTTLE_INPUT   _motor->Raw.ADC_in_ext1 = ADC_buffer[4];  // Throttle input to L431
#define GET_FETU_T   _motor->Raw.MOSu_T = ADC_buffer[3]; //Temperature on PA3
#define GET_FETV_T   _motor->Raw.MOSv_T = ADC_buffer[8];
#define GET_FETW_T   _motor->Raw.MOSw_T = ADC_buffer[9];
#define GET_MOTOR_T  _motor->Raw.Motor_T = ADC_buffer[6];

//#define USE_LR_OBSERVER

/////////////////////Related to ANGLE ESTIMATION////////////////////////////////////////
#define INTERPOLATE_V7_ANGLE
#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_SENSORLESS

#define USE_HFI
#define HFI_VOLTAGE 4.0f
#define HFI_TEST_CURRENT 0.0f
#define HFI_THRESHOLD 4.0f
#define HFI45
#define DEFAULT_HFI_TYPE HFI_TYPE_NONE
//#define DEFAULT_HFI_TYPE HFI_TYPE_45
//#define DEFAULT_HFI_TYPE HFI_TYPE_D
//#define DEFAULT_HFI_TYPE HFI_TYPE_SPECIAL

#define USE_HALL_START
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
//#define LOGGING

#endif /* INC_MX_FOC_IMS_H_ */
