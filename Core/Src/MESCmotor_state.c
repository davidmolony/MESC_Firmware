/*
 * MESCmotor_state.c
 *
 *  Created on: 18 Jul 2020
 *      Author: Lenovo
 */


/* Includes ------------------------------------------------------------------*/
#include "MESCmotor_state.h"

void MESC_Init(){
	MotorState=MOTOR_STATE_IDLE;
	MotorSensorMode=MOTOR_SENSOR_MODE_HALL;
	MotorControlType=MOTOR_CONTROL_TYPE_BLDC;
	MotorDirection=MOTOR_DIRECTION_CLOCKWISE;

}
