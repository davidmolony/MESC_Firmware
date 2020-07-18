/*
 * MESCfoc.c
 *
 *  Created on: 18 Jul 2020
 *      Author: David Molony
 */


/* Includes ------------------------------------------------------------------*/
#include "MESCfoc.h"


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
			int test=8;
			test = GetHallState();

			if((test==6)||(test==7)){
				//no hall sensors detected
				MotorSensorMode==MOTOR_SENSOR_MODE_SENSORLESS;
			}
			else if(test==8){
				MotorState=MOTOR_STATE_ERROR;
			}
			//ToDo add resporting
			else {
				//hall sensors detected
				MotorSensorMode=MOTOR_SENSOR_MODE_HALL;
			}
		 break;

		 case MOTOR_STATE_MEASURING:
			if(Rphase==0){	//Every PWM cycle we enter this function until the resistance measurement has converged at a good value. Once the measurement is complete, Rphase is set, and this is no longer called
				measureResistance();
			break;
			}
			else if(Lphase==0){
				//As per resistance measurement, this will be called until an inductance measurement is converged.
				//Inductance measurement might require a serious reset of the ADC, or calling this function many times per PWM period by resetting the OCR4 register to trigger the ADC successively
				measureInductance();
			break;
			}

		 case ERROR:
			GenerateBreak();	//Generate a break state
			//Now panic and freak out
			break;
	}
}


void V_I_Check(){ // &RawADC1,&RawADC2, &RawADC3 as arguments? Is this the correct use of &pointers? Just need it to look in the buffers filled by the DMA
	//Check currents, voltages are within panic limits
	if((RawADC1[0]>RawCurrLim)|(RawADC2[0]>RawCurrLim)|(RawADC3[0]>RawCurrLim)|(RawADC1[1]>RawVoltLim)){
		GenerateBreak();
		MotorState=ERROR;
	}
}

void GenerateBreak(){
	//Here we set all the PWMoutputs to LOW, without triggering the timerBRK, which should only be set by the hardware comparators, in the case of a shoot-through orother catastrophic event
	//This function means that the timer can be left running, ADCs sampling etc which enables a recovery, or single PWM period break in which the backEMF can be measured directly
	//This function needs implementing and testing before any high current or voltage is applied, otherwise... DeadFETs
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


