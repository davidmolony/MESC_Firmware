/*
 * mxlemming_FOC_IMS.h
 *
 *  Created on: Dec 16, 2022
 *      Author: HPEnvy
 */

#ifndef INC_SQUAREBOARD_H_
#define INC_SQUAREBOARD_H_

//Pick a motor for default
#define QS165V2
//#define ANT_120_70_62KV

#define SHUNT_POLARITY 1.0f

#define ABS_MAX_PHASE_CURRENT 400.0f
#define ABS_MAX_BUS_VOLTAGE 98.0f
#define ABS_MIN_BUS_VOLTAGE 38.0f
#define R_SHUNT 1.0f
#define OPGAIN 0.00075 //Roughly 2200A read capability, tune

#define R_VBUS_BOTTOM 3300.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 150000.0f



#define MAX_ID_REQUEST 100.0f
#define MAX_IQ_REQUEST 100.0f

#define SEVEN_SECTOR		//Normal SVPWM implemented as midpoint clamp. If not defined, you will get 5 sector, bottom clamp
#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.
#define DEADTIME_COMP_V 10
//#define MAX_MODULATION 1.02f
#define CUSTOM_DEADTIME 600 //ns

#define USE_FIELD_WEAKENINGV2
//#define USE_HIGHHOPES_PHASE_BALANCING

#define GET_THROTTLE_INPUT  _motor->Raw.ADC_in_ext1 = hadc1.Instance->JDR3;  // Throttle for IMS board
#define GET_THROTTLE_INPUT2  _motor->Raw.ADC_in_ext2 = ADC1_buffer[3];  // Throttle for IMS board


#define GET_FETU_T _motor->Raw.MOSu_T = ADC2_buffer[2] //CHECK, needs PC4,5, PB0, PB1
#define GET_FETV_T _motor->Raw.MOSv_T = ADC2_buffer[2] //CHECK
#define GET_FETW_T _motor->Raw.MOSw_T = ADC2_buffer[3] //CHECK
#define GET_MOTOR_T _motor->Raw.Motor_T = ADC2_buffer[0]
//#define USE_LR_OBSERVER

/////////////////////Related to ANGLE ESTIMATION////////////////////////////////////////
//#define INTERPOLATE_V7_ANGLE
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
#define ENCODER_E_OFFSET 25000
#define POLE_ANGLE (65536/POLE_PAIRS)

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

//GPIOs for LEDs
#define FASTLED GPIOC
#define FASTLEDIO GPIO_PIN_9
#define FASTLEDIONO 9
#define SLOWLED GPIOB
#define SLOWLEDIO GPIO_PIN_2
#define SLOWLEDIONO 2


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

//HAndbrake assigned pin C10 clashes with SPI for encoder! Don't use both
//#define HANDBRAKE_GPIO GPIOC
//#define HANDBRAKE_PIN GPIO_PIN_10
//#define HANDBRAKE_IONO 10

#define LOGLENGTH 1500

#endif /* INC_SQUAREBOARD_H_ */
