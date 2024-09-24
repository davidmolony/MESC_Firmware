/*
 **
 ******************************************************************************
 * @file           : MESCinterface.c
 * @brief          : Initializing RTOS system and parameters
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Jens Kerrinnes.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 ******************************************************************************
 *In addition to the usual 3 BSD clauses, it is explicitly noted that you
 *do NOT have the right to take sections of this code for other projects
 *without attribution and credit to the source. Specifically, if you copy into
 *copyleft licenced code without attribution and retention of the permissive BSD
 *3 clause licence, you grant a perpetual licence to do the same regarding turning sections of your code
 *permissive, and lose any rights to use of this code previously granted or assumed.
 *
 *This code is intended to remain permissively licensed wherever it goes,
 *maintaining the freedom to distribute compiled binaries WITHOUT a requirement to supply source.
 *
 *This is to ensure this code can at any point be used commercially, on products that may require
 *such restriction to meet regulatory requirements, or to avoid damage to hardware, or to ensure
 *warranties can reasonably be honoured.
 ******************************************************************************/

#include "cmsis_os2.h"
#include "MESCinterface.h"
#include "main.h"
#include "Tasks/init.h"
#include "TTerm/Core/include/TTerm.h"
#include "AXIS/MESCmotor_state.h"
#include "Tasks/task_cli.h"
#include "Tasks/task_can.h"
#include "Tasks/task_overlay.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "TTerm/Core/include/TTerm_cwd.h"


#include <stdio.h>

axis_vars_t axis_vars;



const char TERM_startupText1[] = "\r\n";
const char TERM_startupText2[] = "\r\n[A][X][I][S] - Throttle";
const char TERM_startupText3[] = "\r\n";


void callback(TermVariableDescriptor * var){
	axis_vars.error_count = 0;
}

void populate_vars(){
	can1.node_id = 255;

	//		   | Variable							| MIN		| MAX		| NAME			| DESCRIPTION							| RW			| CALLBACK	| VAR LIST HANDLE
	TERM_addVar(can1.node_id						, 1			, 254		, "node_id"		, "Node ID"								, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(axis_vars.encoder_error_limit		, 0			, 1000		, "enc_err_lim"	, "Encoder error limit"					, VAR_ACCESS_RW	, callback  , &TERM_varList);
	//TERM_addVar(axis_vars.check_pwm_vs_spi			, 0			, 1			, "sanitycheck"	, "Check PWM vs SPI"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	//TERM_addVar(axis_vars.use_spi					, 0			, 1			, "use_spi"		, "Retrieve angle via SPI"				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	//TERM_addVar(axis_vars.use_pwm					, 0			, 1			, "use_pwm"		, "Retrieve angle via PWM"				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(axis_vars.throttle_start			, 0.0f		, 1.0f		, "zero"		, "Throttle zero"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(axis_vars.throttle_end			    , 0.0f		, 1.0f		, "span"		, "Throttle span"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(axis_vars.throttle_offset		    , 0.0f		, 1.0f		, "offset"		, "Throttle offset"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(axis_vars.throttle_threshold	    , 0.0f		, 1.0f		, "threshold"	, "Throttle threshold"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);


}


extern volatile TERMINAL_HANDLE * debug;
uint32_t count=0;




void TASK_CAN_packet_cb(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len){

	switch(id){

		default:
			break;
	}
}


float pi = 3.1415;

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim15;


HAL_StatusTypeDef SpiStatus;



uint8_t has_even_parity(uint16_t x){
	uint32_t count=0;

	for(int i=0; i<16; i++){
		if(x & (1 << i)){
			count ++;
		}
	}

	if(count % 2){
		return 0;
	}else{
		return 1;
	}

}

mt6816_t read_encoder(){

	mt6816_t ret;

	uint16_t SPITxData;
	uint16_t SPIRxData;

	ret.error = MT6816_OK;

	//Generate an NSS
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); //falling edge signals start of transmission
	SPITxData = 0x8300;
	SPIRxData = 0xa5a5;
	SpiStatus = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&SPITxData, (uint8_t*)&SPIRxData, 1, 2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);//Rising edge ends the transmission

	ret.angle = 0;
	ret.angle = (SPIRxData&0x00FF)<<8;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	SPITxData = 0x8400;
	SPIRxData = 0xa5a5;
	SpiStatus = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&SPITxData, (uint8_t*)&SPIRxData, 1, 2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

	ret.angle |= (SPIRxData & 0x00FF);
	ret.angle = ret.angle>>2;

	uint8_t parity_bit = SPIRxData & 0b1;

	ret.status = 0;
	if(has_even_parity(ret.angle) == parity_bit){
		ret.angle = 0;
		ret.error = MT6816_ERROR;
		ret.status |= MT6816_PARITY_ERROR;
	}
	if(SPIRxData & 0b10){  //Check no magnet bit
		ret.angle = 0;
		ret.error = MT6816_ERROR;
		ret.status |= MT6816_NO_MAG;
	}

	return ret;
}


float utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float utils_truncate_number(float number, float min, float max, float deadband) {
	if (number > max) {
		if(number > (max + deadband)){
			return 0.0f;
		}else{
			return max;
		}
	} else if (number < min) {
		if(number < (min - deadband)){
			return 0.0f;
		}else{
			return min;
		}
	}
	return number;
}

void task_encoder(void * argument){

	//Init SPI to the encoder
	HAL_SPI_Init(&hspi1);
	//Init the PW in
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);


	//Start differential analog output timer
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	//Set the output analog to 0V
	htim1.Instance->CCR1 = 0;

	//Start PWM out
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	htim15.Instance->CCR1 = 0;

	while(1){
		bool error_state = false;

		if(axis_vars.use_spi){
			axis_vars.mt6816 = read_encoder();


		//Process the data

			axis_vars.ratioSPI = (float)axis_vars.mt6816.angle/16384.0f;

			if(axis_vars.mt6816.error){
				error_state=true;
			}
		}

		if(axis_vars.use_pwm){
			axis_vars.ratioPWM = (float)htim2.Instance->CCR2/(float)(htim2.Instance->CCR1);

		}

		if(axis_vars.check_pwm_vs_spi){
			if (fabsf(axis_vars.ratioPWM-axis_vars.ratioSPI)>0.05){
				error_state = true;
			}
		}
		bool kill = false;
		//Use second source to verify accuracy
		if(error_state){
		  //Check the PWM and the SPI agree and that the magnet present bit is low
			axis_vars.accumulated_errors++;

			if(axis_vars.encoder_error_limit && axis_vars.error_count >= axis_vars.encoder_error_limit){
				kill = true;
				axis_vars.throttle_raw = 0.0f;
				htim1.Instance->CCR1 = 0;
				htim15.Instance->CCR1 = 0;
			}else{
				axis_vars.error_count++;
			}

		}
		else
		{
			if(axis_vars.encoder_error_limit == 0){
				axis_vars.error_count=0;
			}
			if(axis_vars.error_count){
				axis_vars.error_count--;
			}

			if(axis_vars.error_count == 0){
				htim1.Instance->CCR1 = (uint16_t)(axis_vars.ratioSPI*(float)htim1.Instance->ARR);
				htim15.Instance->CCR1 = (uint16_t)(2000.0f+2000.0f*axis_vars.ratioSPI);

				if(axis_vars.use_spi && axis_vars.use_pwm){
					axis_vars.throttle_raw = axis_vars.ratioSPI;
				}else if(axis_vars.use_pwm == true && axis_vars.use_spi == false){
					axis_vars.throttle_raw = axis_vars.ratioPWM;
				}else if(axis_vars.use_pwm == false && axis_vars.use_spi == true){
					axis_vars.throttle_raw = axis_vars.ratioSPI;
				}
			}else{
				kill = true;
				axis_vars.throttle_raw = 0.0f;
				htim1.Instance->CCR1 = 0;
				htim15.Instance->CCR1 = 0;
			}

		}

		float cal = axis_vars.throttle_raw - axis_vars.throttle_start;
		if(cal < 0.0f){
			cal += 1.0f;
		}
		axis_vars.throttle_z_calib = cal;

		if(cal < axis_vars.throttle_threshold){
			cal = 0.0f;
		}

		axis_vars.throttle_calibrated = utils_truncate_number(utils_map(cal, 0.0f, axis_vars.throttle_end, 0.0f, 1.0f), 0.0f, 1.0f, 0.1f);

		if(kill==false){
			if (axis_vars.throttle_calibrated < axis_vars.throttle_offset) {
				axis_vars.throttle_mapped = utils_map(axis_vars.throttle_calibrated, 0.0f, axis_vars.throttle_offset, -1.0f, 0.0f);
			} else {
				axis_vars.throttle_mapped = utils_map(axis_vars.throttle_calibrated, axis_vars.throttle_offset,	1.0f, 0.0f, 1.0f);
			}
		}else{
			axis_vars.throttle_mapped = 0.0f;
		}

	    vTaskDelay(10);
	}
}


void TASK_CAN_telemetry_fast(TASK_CAN_handle * handle){

	TASK_CAN_add_float(handle	, CAN_ID_ADC1_2_REQ	  	, CAN_BROADCAST, axis_vars.throttle_mapped	, 0.0f	, 0);
}

void TASK_CAN_telemetry_slow(TASK_CAN_handle * handle){

}




void MESCinterface_init(TERMINAL_HANDLE * handle){
	static bool is_init=false;
	if(is_init) return;
	is_init=true;

	populate_vars();

	axis_vars.check_pwm_vs_spi = true;
	axis_vars.use_pwm = true;
	axis_vars.use_spi = true;
	axis_vars.error_count = 0;

	axis_vars.throttle_raw = 0.0f;
	axis_vars.throttle_start = 0.0f;
	axis_vars.throttle_end = 1.0f;
	axis_vars.throttle_offset = 0.0f;
	axis_vars.throttle_threshold = 0.1f;

	if(CMD_varLoad(&null_handle, 0, NULL) == TERM_CMD_EXIT_ERROR){


	}

	TERM_addCommand(CMD_nodes, "nodes", "Node info", 0, &TERM_defaultList);
	TERM_addCommand(CMD_can_send, "can_send", "Send CAN message", 0, &TERM_defaultList);

	REGISTER_apps(&TERM_defaultList);

	xTaskCreate(task_encoder, "tskEncoder", 128, NULL, osPriorityAboveNormal, NULL);


}
