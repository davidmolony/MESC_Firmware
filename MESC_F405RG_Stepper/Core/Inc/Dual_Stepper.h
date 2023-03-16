/*
 * mxlemming_FOC_IMS.h
 *
 *  Created on: Dec 16, 2022
 *      Author: HPEnvy
 */

#ifndef INC_DUAL_STEPPER_H_
#define INC_DUAL_STEPPER_H_
#define STEPPER_MOTOR
//Pick a motor for default
#define PKP246D23A2
#define PWM_FREQUENCY 20000 //Do not go too high on MEGAVESC, gate drivers start to struggle with bootstrap

#define SHUNT_POLARITY 1.0f

#define ABS_MAX_PHASE_CURRENT 4.5f
#define ABS_MAX_BUS_VOLTAGE 25.0f
#define ABS_MIN_BUS_VOLTAGE 8.0f
#define R_SHUNT 0.005f
#define OPGAIN 20.0f

#define R_VBUS_BOTTOM 1000.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 10000.0f

#define DEFAULT_INPUT 0b0001

#define MAX_ID_REQUEST 1.0f
#define MAX_IQ_REQUEST 4.0f

#define SEVEN_SECTOR		//Normal SVPWM implemented as midpoint clamp. If not defined, you will get 5 sector, bottom clamp
#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.
#define DEADTIME_COMP_V 16
#define MAX_MODULATION 0.6f //Use this with 5 sector modulation if you want extra speed


#define USE_FIELD_WEAKENINGV2

#define GET_THROTTLE_INPUT  _motor->Raw.ADC_in_ext1 = ADC2_buffer[3];  // Throttle for Stepper board
#define GET_THROTTLE_INPUT2  _motor->Raw.ADC_in_ext1 = hadc1.Instance->JDR4;  // Throttle for Stepper board

//#define USE_LR_OBSERVER

/////////////////////Related to ANGLE ESTIMATION////////////////////////////////////////
#define INTERPOLATE_V7_ANGLE
#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_SENSORLESS
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_HALL
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_OPENLOOP
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_ENCODER
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_HFI

//#define USE_HFI
#define HFI_VOLTAGE 4.0f
#define HFI_TEST_CURRENT 0.0f
#define HFI_THRESHOLD 4.0f
#define HFI45
#define DEFAULT_HFI_TYPE HFI_TYPE_NONE
//#define DEFAULT_HFI_TYPE HFI_TYPE_45
//#define DEFAULT_HFI_TYPE HFI_TYPE_D
//#define DEFAULT_HFI_TYPE HFI_TYPE_SPECIAL

//#define USE_HALL_START
#define HALL_VOLTAGE_THRESHOLD 1.5f

//#define USE_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS 10
#define ENCODER_E_OFFSET 32000
#define POLE_ANGLE (65536/POLE_PAIRS)
#define DEFAULT_ENCODER_POLARITY 1

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

//GPIOs for LEDs
#define FASTLED GPIOA
#define FASTLEDIO GPIO_PIN_15
#define FASTLEDIONO 15
#define SLOWLED GPIOA
#define SLOWLEDIO GPIO_PIN_12
#define SLOWLEDIONO 12

//GPIO for BRK
#define INV_ENABLE_M1 GPIOB
#define INV_ENABLE_M1_IO GPIO_PIN_2
#define INV_ENABLE_M1_IONO 2

#define INV_ENABLE_M2 GPIOD
#define INV_ENABLE_M2_IO GPIO_PIN_2
#define INV_ENABLE_M2_IONO 2

//GPIO for IC timer //These actually have to be timer compatible pins and
//you must have done something (anything) with the timer in CUBEMX to make it generate the config files
#define IC_TIM_GPIO GPIOB
#define IC_TIM_PIN GPIO_PIN_6
#define IC_TIM_IONO 6
//#define IC_TIMER htim4 //This must be TIM2-TIM5. Untested with other timers
//Assign a use for the input capture timer
//#define IC_TIMER_RCPWM
//#define IC_TIMER_ENCODER
#endif /* INC_MX_FOC_IMS_H_ */
