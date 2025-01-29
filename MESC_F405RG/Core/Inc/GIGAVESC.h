/*
 * mxlemming_FOC_IMS.h
 *
 *  Created on: Dec 16, 2022
 *      Author: HPEnvy
 */

#ifndef INC_GIGAVESC_H_
#define INC_GIGAVESC_H_

//Pick a motor for default
#define MCMASTER_70KV_8080//QS165V2 //
#define PWM_FREQUENCY 20000 //Do not go too high on MEGAVESC, gate drivers start to struggle with bootstrap
#define CUSTOM_DEADTIME 800 //ns, MAX 1500ns! implementation in MESCInit().

#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 50.0f
#define ABS_MAX_BUS_VOLTAGE 45.0f
#define ABS_MIN_BUS_VOLTAGE 18.0f
//#define R_SHUNT 0.00025f
#define R_SHUNT 0.00075f

#define OPGAIN 20.0f*36.0f/(10.0f+10.0f+36.0f)
//Gigavesc Control board v0.1 has a 0.25mR and 10-30-10 divider onto the opamp input to enable some filtering
//MegaVESC has a 0.33mR and a /2 into the opamp. Overall, both basically the same

#define R_VBUS_BOTTOM 1500.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 82000.0f

#define DEFAULT_INPUT 0b1010

#define MAX_ID_REQUEST 2.0f
#define MAX_IQ_REQUEST 50.0f

#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.
#define DEADTIME_COMP_V 16
//#define MAX_MODULATION 1.00f //Use this with 5 sector modulation if you want extra speed
#define MAX_MODULATION 0.90f

#define USE_FIELD_WEAKENINGV2

#define GET_THROTTLE_INPUT  _motor->Raw.ADC_in_ext1 = hadc1.Instance->JDR4;  // Throttle for IMS board
#define GET_THROTTLE_INPUT2 _motor->Raw.ADC_in_ext2 = ADC1_buffer[3];
//#define USE_LR_OBSERVER

/////////////////////Related to ANGLE ESTIMATION////////////////////////////////////////
#define INTERPOLATE_V7_ANGLE
#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_SENSORLESS
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_HALL
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_OPENLOOP
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_ENCODER
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_HFI

#define DEFAULT_STARTUP_SENSOR STARTUP_SENSOR_HFI //	STARTUP_SENSOR_OPENLOOP,STARTUP_SENSOR_HALL,STARTUP_SENSOR_PWM_ENCODER,
#define HFI_VOLTAGE 1.0f
#define HFI_TEST_CURRENT 0.0f
#define HFI_THRESHOLD 0.0f //Defaults to 0.05Vbus if set to 0
//#define DEFAULT_HFI_TYPE HFI_TYPE_NONE
#define DEFAULT_HFI_TYPE HFI_TYPE_45
//#define DEFAULT_HFI_TYPE HFI_TYPE_D
//#define DEFAULT_HFI_TYPE HFI_TYPE_SPECIAL

//#define USE_HALL_START
#define HALL_VOLTAGE_THRESHOLD 2.0f

//#define USE_SPI_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS 10
#define ENCODER_E_OFFSET 32000
#define POLE_ANGLE (65536/POLE_PAIRS)
#define DEFAULT_ENCODER_POLARITY 1

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

//GPIOs for LEDs
#define FASTLED GPIOB
#define FASTLEDIO GPIO_PIN_5
#define FASTLEDIONO 5
#define SLOWLED GPIOB
#define SLOWLEDIO GPIO_PIN_7
#define SLOWLEDIONO 7

//GPIO for IC timer //These actually have to be timer compatible pins and
//you must have done something (anything) with the timer in CUBEMX to make it generate the config files
#define IC_TIM_GPIO GPIOB
#define IC_TIM_PIN GPIO_PIN_6
#define IC_TIM_IONO 6
#define IC_TIMER htim4 //This must be TIM2-TIM5. Untested with other timers
//Assign a use for the input capture timer
//#define IC_TIMER_RCPWM
#define IC_TIMER_ENCODER
#endif /* INC_MX_FOC_IMS_H_ */
