/*
 * MESCfoc.c
 *
 *  Created on: 18 Jul 2020
 *      Author: David Molony
 */


/* Includes ------------------------------------------------------------------*/
#include "MESCfoc.h"
#include "MESCmotor_state.h"
#include "MESChw_setup.h"

void fastLoop(){//Call this directly from the ADC callback IRQ
V_I_Check(); //Run the current and voltage checks
switch(MotorState){
		 case MOTOR_STATE_SENSORLESS_RUN:
			//Call the observer
			//Call the current and phase controller
			//Write the PWM values
			break;

		 case MOTOR_STATE_HALL_RUN:
			//Get the current position from HallTimer
			//Call the current and phase controller
			//Write the PWM values
			break;

		 case MOTOR_STATE_HALL_NEAR_STATIONARY:
			//Call GetHallState
			//Call the BLDC discrete controller - Override the normal current controller, this is 6 step DC only
			//Write the PWM values
			break;

		 case MOTOR_STATE_OPEN_LOOP_STARTUP:
			//Same as open loop
			//Write the PWM values
			break;

		 case MOTOR_STATE_OPEN_LOOP_TRANSITION:
			//Run open loop
			//Run observer
			//RunFOC
			//Weighted average of the outputs N PWM cycles
			//Write the PWM values
			break;

		 case MOTOR_STATE_IDLE:
			  // Do basically nothing
			  //ToDo Set PWM to no output state
		  break;

		 case MOTOR_STATE_DETECTING:
			 ;
			int test=8;
			test = GetHallState();

			if((test==6)||(test==7)){
				//no hall sensors detected
				MotorSensorMode=MOTOR_SENSOR_MODE_SENSORLESS;
			}
			else if(test==8){
				MotorState=MOTOR_STATE_ERROR;
			}
			//ToDo add reporting
			else {
				//hall sensors detected
				MotorSensorMode=MOTOR_SENSOR_MODE_HALL;
			}
		 break;

		 case MOTOR_STATE_MEASURING:
			if(motor.Rphase==0){	//Every PWM cycle we enter this function until the resistance measurement has converged at a good value. Once the measurement is complete, Rphase is set, and this is no longer called
				measureResistance();
			break;
			}
			else if(motor.Lphase==0){
				//As per resistance measurement, this will be called until an inductance measurement is converged.
				//Inductance measurement might require a serious reset of the ADC, or calling this function many times per PWM period by resetting the OCR4 register to trigger the ADC successively
				measureInductance();
			break;
			}

		 case MOTOR_STATE_ERROR:
			GenerateBreak();	//Generate a break state
			//Now panic and freak out
			break;
		 case MOTOR_STATE_ALIGN:
			//Turn on at a given voltage at electricalangle0;
			break;
		 case MOTOR_STATE_RECOVERING:
			//No clue so far. Read the phase voltages and determine position and attempt to restart?
			//Should already be in break state, and should stay there...
			break;
	}
}


void V_I_Check(){ // &RawADC1,&RawADC2, &RawADC3 as arguments? Is this the correct use of &pointers? Just need it to look in the buffers filled by the DMA
	//Check currents, voltages are within panic limits
	if((measurement_buffers.RawADC[0][0]>motor.RawCurrLim)|(measurement_buffers.RawADC[1][0]>motor.RawCurrLim)|(measurement_buffers.RawADC[2][0]>motor.RawCurrLim)|(measurement_buffers.RawADC[0][1]>motor.RawVoltLim)){
		GenerateBreak();
		MotorState=ERROR;
	}
}

void GenerateBreak(){
	//Here we set all the PWMoutputs to LOW, without triggering the timerBRK, which should only be set by the hardware comparators, in the case of a shoot-through orother catastrophic event
	//This function means that the timer can be left running, ADCs sampling etc which enables a recovery, or single PWM period break in which the backEMF can be measured directly
	//This function needs implementing and testing before any high current or voltage is applied, otherwise... DeadFETs
	phU_Break();
	phV_Break();
	phW_Break();
}

int GetHallState(){


	int hallState=0;
	hallState=((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))|((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))<<1)|((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))<<2));
	switch(hallState)
		{
			case 0:
				return 7; //7 is the no hall sensor detected state (all low)
				break;
			case 7:
				return 6; //6 is the no hall sensor detected state (all high)
				break;
//Implement the hall table order here, depending how the hall sensors are configured
			case 1:
				return 0;
				break;
			case 3:
				return 1;
				break;
			case 2:
				return 2;
				break;
			case 6:
				return 3;
				break;
			case 4:
				return 4;
				break;
			case 5:
				return 5;
				break;
			default:
				return 8;
				break;
		}
}
void measureResistance(){

}
void measureInductance(){

}


uint32_t tmpccmrx;	//Temporary buffer which is used to turn on/off phase PWMs
//Turn all phase U FETs off, Tristate the HBridge output - For BLDC mode mainly, but also used for measuring, software fault detection and recovery
//ToDo TEST THOROUGHLY The register manipulations for the break functions were used previously on an STM32F042K6 for my first BLDC drive, on TIM1, which should be identical, but definitely needs checking
void phU_Break(){
	tmpccmrx = htim1.Instance->CCMR1;
	tmpccmrx &= ~TIM_CCMR1_OC1M;
	tmpccmrx &= ~TIM_CCMR1_CC1S;
	tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
	htim1.Instance->CCMR1 = tmpccmrx;
	htim1.Instance->CCER &= ~TIM_CCER_CC1E;  //disable
	htim1.Instance->CCER &= ~TIM_CCER_CC1NE;  //disable
}
//Basically un-break phase U, opposite of above...
void phU_Enable(){
	tmpccmrx = htim1.Instance->CCMR1;
	tmpccmrx &= ~TIM_CCMR1_OC1M;
	tmpccmrx &= ~TIM_CCMR1_CC1S;
	tmpccmrx |= TIM_OCMODE_PWM1;
	htim1.Instance->CCMR1 = tmpccmrx;
	htim1.Instance->CCER |= TIM_CCER_CC1E;   //enable
	htim1.Instance->CCER |= TIM_CCER_CC1NE;   //enable
}

void phV_Break(){
	tmpccmrx = htim1.Instance->CCMR1;
	tmpccmrx &= ~TIM_CCMR1_OC2M;
	tmpccmrx &= ~TIM_CCMR1_CC2S;
	tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE<<8;
	htim1.Instance->CCMR1 = tmpccmrx;
	htim1.Instance->CCER &= ~TIM_CCER_CC2E;  //disable
	htim1.Instance->CCER &= ~TIM_CCER_CC2NE;  //disable
}

void phV_Enable(){
	tmpccmrx = htim1.Instance->CCMR1;
	tmpccmrx &= ~TIM_CCMR1_OC2M;
	tmpccmrx &= ~TIM_CCMR1_CC2S;
	tmpccmrx |= TIM_OCMODE_PWM1<<8;
htim1.Instance->CCMR1 = tmpccmrx;
htim1.Instance->CCER |= TIM_CCER_CC2E;   //enable
htim1.Instance->CCER |= TIM_CCER_CC2NE;   //enable
}

void phW_Break(){
	   tmpccmrx = htim1.Instance->CCMR2;
	   tmpccmrx &= ~TIM_CCMR2_OC3M;
	   tmpccmrx &= ~TIM_CCMR2_CC3S;
	   tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
	   htim1.Instance->CCMR2 = tmpccmrx;
	   htim1.Instance->CCER &= ~TIM_CCER_CC3E;  //disable
	   htim1.Instance->CCER &= ~TIM_CCER_CC3NE;  //disable
}

void phW_Enable(){
  	tmpccmrx = htim1.Instance->CCMR2;
    tmpccmrx &= ~TIM_CCMR2_OC3M;
    tmpccmrx &= ~TIM_CCMR2_CC3S;
    tmpccmrx |= TIM_OCMODE_PWM1;
    htim1.Instance->CCMR2 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC3E;   //enable
    htim1.Instance->CCER |= TIM_CCER_CC3NE;   //enable
}
