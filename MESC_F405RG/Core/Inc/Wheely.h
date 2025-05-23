/*
 * Wheely PCB
 *
 *  Created on: Aug 2024
 *      Author: HPEnvy
 */

#ifndef INC_WHEELY_H_
#define INC_WHEELY_H_
//Pick a motor for default
#define Q4120_700KV//Q3513_700KV//MCMASTER_70KV_8080//QS165//CA120//
#define PWM_FREQUENCY 20000
#define CUSTOM_DEADTIME 200 //ns, MAX 1500ns! implementation in MESCInit().

#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 75.0f //We set this as the board abs max, and the firmware sets the value actually used depending on the input setpoints with this as a maximum.
#define ABS_MAX_BUS_VOLTAGE 48.0f
#define ABS_MIN_BUS_VOLTAGE 12.0f
#define R_SHUNT 0.001f
#define OPGAIN 20.0f
#define DAC_REF

#define R_VBUS_BOTTOM 3300.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 82000.0f

#define MAX_ID_REQUEST 2.0f
#define MAX_IQ_REQUEST 10.0f
#define MIN_IQ_REQUEST -10.0f
#define DEFAULT_CONTROL_MODE MOTOR_CONTROL_MODE_TORQUE
#define ADC1OOR 4090


#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.
#define DEADTIME_COMP_V 5
//#define MAX_MODULATION 1.10f //Use this with 5 sector modulation if you want extra speed
//Inputs
#define GET_THROTTLE_INPUT 		_motor->Raw.ADC_in_ext2 = 0.9f*_motor->Raw.ADC_in_ext2 + 0.1f*ADC1_buffer[0];  // Throttle for Wheely on PA3
#define GET_THROTTLE_INPUT2 	0 //There is no second throttle input

#define GET_FETU_T 			_motor->Raw.MOSu_T = hadc2.Instance->JDR4; //Temperature on PC5, ADC15

#define GET_MOTOR_T _motor->Raw.Motor_T = ADC2_buffer[3]; //MotorT for WHEELY on PB1
#define USE_FIELD_WEAKENINGV2

/////////////////////Related to ANGLE ESTIMATION////////////////////////////////////////
#define INTERPOLATE_V7_ANGLE
#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_SENSORLESS
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_HALL
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_OPENLOOP
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_ENCODER
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_HFI

#define DEFAULT_STARTUP_SENSOR STARTUP_SENSOR_OPENLOOP  //	STARTUP_SENSOR_HFI,STARTUP_SENSOR_OPENLOOP,STARTUP_SENSOR_HALL,STARTUP_SENSOR_PWM_ENCODER,
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
#define ENCODER_E_OFFSET 25000

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

#define FASTLED GPIOB
#define FASTLEDIO GPIO_PIN_7
#define FASTLEDIONO 7
#define SLOWLED GPIOB
#define SLOWLEDIO GPIO_PIN_5
#define SLOWLEDIONO 5

//#define SAFE_START_DEFAULT 0


#define LOGGING

//GPIO for IC timer //These actually have to be timer compatible pins and
//you must have done something (anything) with the timer in CUBEMX to make it generate the config files
#define IC_TIM_GPIO GPIOB
#define IC_TIM_PIN GPIO_PIN_6
#define IC_TIM_IONO 6
//#define IC_TIMER htim4 //This must be TIM2-TIM5. Untested with other timers
//Assign a use for the input capture timer
#define IC_TIMER_RCPWM
//#define IC_TIMER_ENCODER


#endif /* INC_WHEELY_H_ */
