/*
 * Q6F_minidesk board. Small board for dev work
 *
 *  Created on: NOV 2024
 *      Author: HPEnvy
 */

#ifndef INC_Q6F_H_
#define INC_Q6F_H_
//Pick a motor for default
#define Q4120_700KV
#define PWM_FREQUENCY 20000 //required stable min 70kerpm = 1166eHz. This gives 17 PWMs per sin wave... probably OK but test with 25k
#define CUSTOM_DEADTIME 250 //ns, Rgate 8.2R up, 1Rdown, dead time should be minimal. Check.

#define SHUNT_POLARITY 1.0f

#define ABS_MAX_PHASE_CURRENT 150.0f //We set this as the board abs max, and the firmware sets the value actually used depending on the input setpoints with this as a maximum.
#define ABS_MAX_BUS_VOLTAGE 55.0f	//Limit is the 60V DCDCs
#define ABS_MIN_BUS_VOLTAGE 12.0f	//DCDCs will not be able to create 12V line below this
#define R_SHUNT 0.0005f
#define OPGAIN 20.0f

#define R_VBUS_BOTTOM 2200.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 39000.0f

#define MAX_ID_REQUEST 20.0f
#define MAX_IQ_REQUEST 50.0f
#define MIN_IQ_REQUEST -50.0f
#define DEFAULT_CONTROL_MODE MOTOR_CONTROL_MODE_TORQUE
#define ADC1OOR 4090


#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.
#define DEADTIME_COMP_V 10
//#define MAX_MODULATION 1.10f //Use this with 5 sector modulation if you want extra speed
//Inputs
#define GET_THROTTLE_INPUT2 	0;//No second throttle
#define GET_THROTTLE_INPUT 		_motor->Raw.ADC_in_ext1 = 0.99f*_motor->Raw.ADC_in_ext1 + 0.01f*hadc1.Instance->JDR3;  // Throttle2 for Run on Pa4


#define GET_FETU_T 			_motor->Raw.MOSu_T = ADC2_buffer[3]; //Temperature on PB1, ADC2-9, buffer[3]

#define GET_MOTOR_T _motor->Raw.Motor_T = ADC1_buffer[4]; //MotorT for Q6 on PA7, ADC1-7, buffer[4]

#define GET_DCDC_VRAW			_motor->Raw.MOSu_T = ADC2_buffer[1]; //DCDC V on PC5, ADC2-15, buffer[1]
#define GET_DCDC_IRAW			_motor->Raw.MOSu_T = ADC2_buffer[0]; //DCDC V on PC4, ADC2-14, buffer[0]

#define R_VDCDCAUX_BOTTOM 4700.0f //Aux DCDC voltage dividers
#define R_VDCDCAUX_TOP 15000.0f
#define DCDC_SENS_GAIN 1.0f/(50f*0.01f) //Amp per Volt

#define GET_DCDC_VOLT (3.3f*(R_VDCDCAUX_TOP+R_VDCDCAUX_BOTTOM)*GET_DCDC_VRAW)/(R_VDCDCAUX_BOTTOM * 4096.0f);
#define GET_DCDC_AMP (DCDC_SENS_GAIN*3.3f*GET_DCDC_IRAW)/(4096.0f);


//#define USE_FIELD_WEAKENING
#define USE_FIELD_WEAKENINGV2

//#define USE_LR_OBSERVER

/////////////////////Related to ANGLE ESTIMATION////////////////////////////////////////
#define INTERPOLATE_V7_ANGLE
#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_SENSORLESS
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_HALL
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_OPENLOOP
//#define DEFAULT_SENSOR_MODE MOTOR_SENSOR_MODE_ENCODER

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
#define ENCODER_E_OFFSET 25000

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.

#define FASTLED GPIOB
#define FASTLEDIO GPIO_PIN_4
#define FASTLEDIONO 4
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

//#define KILLSWITCH_GPIO GPIOB
//#define KILLSWITCH_PIN GPIO_PIN_3
//#define KILLSWITCH_IONO 3

//#define REVERSE_GPIO GPIOB		//Define to enable
#define REVERSE_GPIO_PIN GPIO_PIN_11
#define REVERSE_IONO 11

//#define SIDESTAND_GPIO GPIOB
#define SIDESTAND_GPIO_PIN GPIO_PIN_10
#define SIDESTAND_IONO 10

//#define BRAKE_GPIO GPIOA
#define BRAKE_GPIO_PIN GPIO_PIN_5
#define BRAKE_IONO 5

#endif /* INC_Q6F_H_ */
