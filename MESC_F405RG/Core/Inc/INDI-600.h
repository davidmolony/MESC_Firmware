/*
 * mxlemming_FOC_IMS.h
 *
 *  Created on: Dec 16, 2022
 *      Author: HPEnvy
 */

#ifndef INC_INDI_600_H_
#define INC_INDI_600_H_

//Pick a motor for default
#define QS138_90H

#define SHUNT_POLARITY -1.0f
#define MISSING_VCURRSENSOR

#define ABS_MAX_PHASE_CURRENT 1200.0f
#define ABS_MAX_BUS_VOLTAGE 96.0f
#define ABS_MIN_BUS_VOLTAGE 38.0f
#define R_SHUNT 0.0009f //This is 1800A measuring range???
#define OPGAIN 1 //This is a phase hall sensor design

#define R_VBUS_BOTTOM 2000.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 150000.0f



#define MAX_ID_REQUEST 2.0f
#define MAX_IQ_REQUEST 100.0f //Can overwrite this from the terminal

#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.
#define DEADTIME_COMP_V 15
//#define MAX_MODULATION 1.00f //Normally 0.95 used
#define CUSTOM_DEADTIME 1000 //ns

#define USE_FIELD_WEAKENINGV2
//#define USE_HIGHHOPES_PHASE_BALANCING

#define GET_THROTTLE_INPUT  _motor->Raw.ADC_in_ext1 = hadc1.Instance->JDR4;  // Throttle for IMS board
#define GET_THROTTLE_INPUT2  _motor->Raw.ADC_in_ext2 = ADC1_buffer[3];  // Throttle for IMS board


#define GET_FETU_T _motor->Raw.MOSu_T = ADC1_buffer[0]; //Temperature on PA3
#define GET_MOTOR_T _motor->Raw.Motor_T = hadc2.Instance->JDR3;
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
#define HFI_THRESHOLD 2.5f
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

//GPIOs for LEDs
#define FASTLED GPIOB
#define FASTLEDIO GPIO_PIN_0
#define FASTLEDIONO 0
#define SLOWLED GPIOB
#define SLOWLEDIO GPIO_PIN_1
#define SLOWLEDIONO 1


//GPIO for IC timer //These actually have to be timer compatible pins and
//you must have done something (anything) with the timer in CUBEMX to make it generate the config files
#define IC_TIM_GPIO GPIOB
#define IC_TIM_PIN GPIO_PIN_6
#define IC_TIM_IONO 6
#define IC_TIMER htim4 //This must be TIM2-TIM5. Untested with other timers
//Assign a use for the input capture timer
//#define IC_TIMER_RCPWM
#define IC_TIMER_ENCODER

//#define KILLSWITCH_GPIO GPIOD
//#define KILLSWITCH_PIN GPIO_PIN_2
//#define KILLSWITCH_IONO 2
#define BRAKE_DIGITAL_GPIO GPIOB
#define BRAKE_DIGITAL_PIN GPIO_PIN_2
#define BRAKE_DIGITAL_IONO 2
#define BRAKE_DIGITAL_CURRENT 50 //Default amps of on-off brake

#define HANDBRAKE_GPIO GPIOB
#define HANDBRAKE_PIN GPIO_PIN_2
#define HANDBRAKE_IONO 2

#define LOGLENGTH 1500

#endif /* INC_INDI_600_H_ */
