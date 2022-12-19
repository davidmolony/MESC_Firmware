/*
 * MX_FOC_IMS.h
 *
 *  Created on: Dec 16, 2022
 *      Author: D Molony
 */

#ifndef INC_MX_FOC_IMS_H_
#define INC_MX_FOC_IMS_H_

//Pick a motor for default
#define MCMASTER_70KV_8080

//#define PWM_FREQUENCY 38000

#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 250.0f
#define ABS_MAX_BUS_VOLTAGE 45.0f
#define ABS_MIN_BUS_VOLTAGE 38.0f
#define R_SHUNT 0.00025f
#define OPGAIN 20.0f*30.0f/(4.7f+4.7f+30.0f) //Control board v0.1 has a 4.7-30-4.7 divider onto the opamp input to enable some filtering

#define R_VBUS_BOTTOM 3300.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 100000.0f

#define DEFAULT_INPUT	0b0010 //0b...wxyz where w is UART, x is RCPWM, y is ADC1 z is ADC2

#define MAX_ID_REQUEST 2.0f
#define MAX_IQ_REQUEST 40.0f

#define SEVEN_SECTOR		//Normal SVPWM implemented as midpoint clamp. If not defined, you will get 5 sector, bottom clamp
#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.
#define DEADTIME_COMP_V 10

//#define USE_FIELD_WEAKENINGV2

#define GET_THROTTLE_INPUT   _motor->Raw.ADC_in_ext1 = ADC_buffer[4];  // Throttle input to L431


//#define USE_LR_OBSERVER

/////////////////////Related to ANGLE ESTIMATION////////////////////////////////////////
#define INTERPOLATE_V7_ANGLE

//#define USE_HFI

#define USE_HALL_START
#define HALL_VOLTAGE_THRESHOLD 1.5f

//#define USE_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS 10
#define ENCODER_E_OFFSET 25000
#define POLE_ANGLE (65536/POLE_PAIRS)

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

#define LOGGING

#endif /* INC_MX_FOC_IMS_H_ */
