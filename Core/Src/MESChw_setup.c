/*
 * MESChw_setup.c
 *
 *  Created on: 18 Jul 2020
 *      Author: Lenovo
 */


/* Includes ------------------------------------------------------------------*/
#include "MESChw_setup.h"



void motor_init(){
	motor.Rphase=0;		//We init at 0 to trigger the measurer to get the vals
	motor.Lphase=0;		//We init at 0 to trigger the measurer to get the vals
	motor.uncertainty=1;
	motor.RawCurrLim=3000;
	motor.RawVoltLim=2303;
}

void hw_init(){
	g_hw_setup.Rshunt=0.001;
	g_hw_setup.RIphPU=4700;
	g_hw_setup.RIphSR=150;
	g_hw_setup.RVBB=1500;
	g_hw_setup.RVBT=47000;
	g_hw_setup.OpGain=16; //Can this be inferred from the HAL declaration?
	g_hw_setup.VBGain=g_hw_setup.RVBB/(g_hw_setup.RVBB+g_hw_setup.RVBT);
	g_hw_setup.Igain=g_hw_setup.Rshunt*g_hw_setup.OpGain*g_hw_setup.RIphPU/(g_hw_setup.RIphPU+g_hw_setup.RIphSR);

}
