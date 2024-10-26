/*
 * mxlemming_FOC_IMS.h
 *
 *  Created on: Dec 16, 2022
 *      Author: HPEnvy
 */

#ifndef INC_CL700_V0_3_H_
#define INC_CL700_V0_3_H_

//Pick a motor for default
#define QS138_90H

#define SHUNT_POLARITY 	-1.0f

#define ABS_MAX_PHASE_CURRENT 1000.0f	
#define ABS_MAX_BUS_VOLTAGE 90.0f
#define ABS_MIN_BUS_VOLTAGE 24.0f
#define R_SHUNT 0.00005f
#define OPGAIN 20.0f

#define R_VBUS_BOTTOM 2000.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 150000.0f



#define MAX_ID_REQUEST 2.0f
#define MAX_IQ_REQUEST 50.0f

#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.
#define DEADTIME_COMP_V 10
//#define MAX_MODULATION 1.02f

#define USE_FIELD_WEAKENINGV2
#define USE_HIGHHOPES_PHASE_BALANCING

#define GET_THROTTLE_INPUT  _motor->Raw.ADC_in_ext1 = hadc1.Instance->JDR4;  // Throttle for CL700v3
#define GET_THROTTLE_INPUT2  _motor->Raw.ADC_in_ext2 = ADC1_buffer[3];  // Second throttle for CL700v3

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

//#define USE_HALL_START
#define HALL_VOLTAGE_THRESHOLD 1.5f

//#define USE_SPI_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS 10
#define ENCODER_E_OFFSET 25000
#define POLE_ANGLE (65536/POLE_PAIRS)

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

//GPIOs for LEDs
#define FASTLED GPIOB
#define FASTLEDIO GPIO_PIN_3
#define FASTLEDIONO 3
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
#define IC_TIMER_RCPWM
//#define IC_TIMER_ENCODER

//#define KILLSWITCH_GPIO GPIOD
//#define KILLSWITCH_PIN GPIO_PIN_2
//#define KILLSWITCH_IONO 2

#endif /* INC_MX_FOC_IMS_H_ */
