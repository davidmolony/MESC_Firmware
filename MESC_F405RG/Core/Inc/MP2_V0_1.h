/*
 * MP2_V0_1.h
 *
 *  Created on: Dec 16, 2022
 *      Author: HPEnvy
 */

#ifndef INC_MP2_V0_1_H_
#define INC_MP2_V0_1_H_
//Pick a motor for default
#define QS165//MCMASTER_70KV_8080//QS165//CA120//
#define PWM_FREQUENCY 20000
#define CUSTOM_DEADTIME 800 //ns, MAX 1500ns! implementation in MESCInit().

#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 400.0f //We set this as the board abs max, and the firmware sets the value actually used depending on the input setpoints with this as a maximum.
#define ABS_MAX_BUS_VOLTAGE 104.0f
#define ABS_MIN_BUS_VOLTAGE 38.0f
#define R_SHUNT 0.00033f
#define OPGAIN 10.5f

#define R_VBUS_BOTTOM 3300.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 150000.0f //150V range for MP2 DFN

#define MAX_ID_REQUEST 2.0f
#define MAX_IQ_REQUEST 10.0f
#define MIN_IQ_REQUEST -10
#define DEFAULT_CONTROL_MODE MOTOR_CONTROL_MODE_TORQUE
#define ADC1OOR 4094


#define SEVEN_SECTOR		//Normal SVPWM implemented as midpoint clamp. If not defined, you will get 5 sector, bottom clamp
#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.
#define DEADTIME_COMP_V 10
//#define MAX_MODULATION 1.05f //Use this with 5 sector modulation if you want extra speed

//Inputs
#define GET_FETU_T 	_motor->Raw.MOSu_T = 	0.99f * _motor->Raw.MOSu_T + 0.01f*ADC2_buffer[3] //Temperature on PB1
#define GET_MOTOR_T _motor->Raw.Motor_T = ADC1_buffer[4]
#define GET_EXT_ADC_FAILSAFE _motor->Raw.ext_adc_killswitch = ADC1_buffer[3] // PA06 on the F405 pill
#define KILLSWITCH_ANALOG_CUTOFF 360 // arbitrary value due to voltage divider on PA06

//Testing
#define HIGH_ADC_SAFECOUNT 40
#define HIGH_ADC_THRESHOLD 1300

 // Throttle for MP2 with F405 pill
#define GET_THROTTLE_INPUT 	_motor->Raw.ADC_in_ext1 = 0.99f*_motor->Raw.ADC_in_ext1 + 0.01f*hadc1.Instance->JDR3;

#ifndef GET_EXT_ADC_FAILSAFE
// You can NOT define this if
//#define GET_THROTTLE_INPUT2 	_motor->Raw.ADC_in_ext2 = 0.99f*_motor->Raw.ADC_in_ext2 + 0.01f*hadc1.Instance->JDR3;
#endif

//#define USE_FIELD_WEAKENING
#define USE_FIELD_WEAKENINGV2

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
#define HFI_THRESHOLD 0.0f
#define HFI45
#define DEFAULT_HFI_TYPE HFI_TYPE_NONE
//#define DEFAULT_HFI_TYPE HFI_TYPE_45
//#define DEFAULT_HFI_TYPE HFI_TYPE_D
//#define DEFAULT_HFI_TYPE HFI_TYPE_SPECIAL

#define USE_HALL_START
#define HALL_VOLTAGE_THRESHOLD 2.0f

//#define USE_SPI_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS 10
#define ENCODER_E_OFFSET 25000
#define POLE_ANGLE (65536/POLE_PAIRS)

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

#define FASTLED GPIOC
#define FASTLEDIO GPIO_PIN_12
#define FASTLEDIONO 12
#define SLOWLED GPIOC
#define SLOWLEDIO GPIO_PIN_9
#define SLOWLEDIONO 9

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

#define KILLSWITCH_GPIO GPIOB
#define KILLSWITCH_PIN GPIO_PIN_3
#define KILLSWITCH_IONO 3

#endif /* INC_MP2_V0_1_H_ */
