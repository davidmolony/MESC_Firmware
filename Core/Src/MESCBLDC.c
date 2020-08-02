/*
 * MESCBLDC.c
 *
 *  Created on: 25 Jul 2020
 *      Author: Lenovo
 */

#include "MESCBLDC.h"
#include "MESCfoc.h"
#include "MESCmotor_state.h"
#include "MESChw_setup.h"

extern TIM_HandleTypeDef htim1;


void BLDCInit(){
	BLDCVars.ReqCurrent=0;					//Start the motor at 0 current
	BLDCVars.BLDCduty=0;
	BLDCVars.CurrentChannel=0;
	BLDCVars.pGain=1023*motor.Rphase/48; 	//wtf should I set the gain as by default... V/Amp error...Perhaps base it on Rphase and the bus voltage (nominally 48V)? But we don;t know the exact bus voltage yet...
	BLDCVars.iGain=BLDCVars.pGain;			//Initially, let's just make the iGain the same as the pGain, so after 1 second their contributions will be equal.
	BLDCVars.BLDCEstate=GetHallState();
	BLDCState=BLDC_FORWARDS;

}


void BLDCCommuteHall(){
int CurrentHallState=GetHallState(); //Borrow the hall state detection from the FOC system

static int LastHallState=7;	//Initialise the LastHallState at a value that means it will call the commutation and correctly set the current measurement channel, avoiding a runaway on the PI loop

if(BLDCState==BLDC_FORWARDS){
	if(!(LastHallState==CurrentHallState)){
		BLDCVars.BLDCEstate=(CurrentHallState+1)%6;
		writeBLDC();	//Write the PWM values for the next state to generate forward torque
		LastHallState=CurrentHallState;		//Avoid repeatedly writing the registers
}

}
else if(BLDCState==BLDC_BACKWARDS){
	if(!(LastHallState==CurrentHallState)){
		BLDCVars.BLDCEstate=(CurrentHallState+5)%6;
		writeBLDC();	//Write the PWM values for the previous state to generate reverse torque
		LastHallState=CurrentHallState;
	}
}
else if(BLDCState==BLDC_BRAKE){
	//ToDo Logic to always be on synch or hanging 1 step in front or behind...

		if(((CurrentHallState-LastHallState)%6)>1){
			BLDCVars.BLDCEstate=(CurrentHallState-1)%6;
			writeBLDC();
			LastHallState=CurrentHallState;
		}
		else if(((CurrentHallState-LastHallState)%6)<-1){
			BLDCVars.BLDCEstate=(CurrentHallState+1)%6;
			writeBLDC();
			LastHallState=CurrentHallState;
		}

}
else{
//Disable the drivers, freewheel
phU_Break();
phV_Break();
phW_Break();
}
}


void BLDCCurrentController(){
//Implement a simple PI controller
	static float CurrentError=0;
	static float CurrentIntegralError=0;
	static int Duty=0;
	CurrentError=(BLDCVars.ReqCurrent-measurement_buffers.ConvertedADC[BLDCVars.CurrentChannel][0]);
	CurrentIntegralError=CurrentIntegralError + CurrentError*0.000027; //37kHz PWM, so the integral portion should be multiplied by 1/37k before accumulating

	Duty=(int)(CurrentError*BLDCVars.pGain + CurrentIntegralError*BLDCVars.iGain);
	
	if(Duty>1023){
		Duty=1023;
	}
	else if(Duty<0){
		Duty=0;
	}

	BLDCVars.BLDCduty=Duty;

}

void writeBLDC(){
	switch(BLDCVars.BLDCEstate){
	case 0:
		//disable phase first
		phW_Break();
		//WritePWM values
		htim1.Instance->CCR1=BLDCVars.BLDCduty;
		htim1.Instance->CCR2=0;
		phU_Enable();
		phV_Enable();
		BLDCVars.CurrentChannel=1; //Write the field into which the lowside current will flow, to be retrieved from the FOC_measurement_vars
		break;

	case 1:
		phV_Break();
		htim1.Instance->CCR1=BLDCVars.BLDCduty;
		htim1.Instance->CCR3=0;
		phU_Enable();
		phW_Enable();
		BLDCVars.CurrentChannel=2;
		break;

	case 2:
		phU_Break();
		htim1.Instance->CCR2=BLDCVars.BLDCduty;
		htim1.Instance->CCR3=0;
		phV_Enable();
		phW_Enable();
		BLDCVars.CurrentChannel=2;
		break;

	case 3:
		phW_Break();
		htim1.Instance->CCR1=0;
		htim1.Instance->CCR2=BLDCVars.BLDCduty;
		phU_Enable();
		phV_Enable();
		BLDCVars.CurrentChannel=0;
		break;

	case 4:
		phV_Break();
		htim1.Instance->CCR1=0;
		htim1.Instance->CCR3=BLDCVars.BLDCduty;
		phU_Enable();
		phW_Enable();
		BLDCVars.CurrentChannel=0;
		break;

	case 5:
		phU_Break();
		htim1.Instance->CCR2=0;
		htim1.Instance->CCR3=BLDCVars.BLDCduty;
		phV_Enable();
		phW_Enable();
		BLDCVars.CurrentChannel=1;
		break;

	}
}
