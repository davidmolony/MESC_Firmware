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


void * TASK_CAN_allocate_node(TASK_CAN_handle * handle, TASK_CAN_node * node){
	if(memcmp(node->short_name, "ESC", 3)==0){
		node->type = NODE_TYPE_ESC;
		node->data = pvPortMalloc(sizeof(esc_data));
		((esc_data*)node->data)->node = node;
	}
	return NULL;
}

void * TASK_CAN_free_node(TASK_CAN_handle * handle, TASK_CAN_node * node){
	vPortFree(node->data);
	return NULL;
}




void populate_vars(){
	can1.node_id = 255;

	//		   | Variable							| MIN		| MAX		| NAME			| DESCRIPTION							| RW			| CALLBACK	| VAR LIST HANDLE
	TERM_addVar(can1.node_id						, 1			, 254		, "node_id"		, "Node ID"								, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(axis_vars.encoder_error_limit		, 0			, 1000		, "enc_err_lim"	, "Encoder error limit"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(axis_vars.check_pwm_vs_spi			, 0			, 1			, "sanitycheck"	, "Check PWM vs SPI"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(axis_vars.use_spi					, 0			, 1			, "use_spi"		, "Retrieve angle via SPI"				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(axis_vars.use_pwm					, 0			, 1			, "use_spi"		, "Retrieve angle via PWM"				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
}


extern volatile TERMINAL_HANDLE * debug;
uint32_t count=0;

void TASK_CAN_packet_esc(esc_data * esc, uint32_t id, uint8_t* data, uint32_t len){
	switch(id){
		case CAN_ID_ADC1_2_REQ:
			esc->adc1 = PACK_buf_to_float(data);
			esc->adc2 = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_SPEED:
			esc->speed = PACK_buf_to_float(data);
			break;
		case CAN_ID_BUS_VOLT_CURR:
			esc->bus_voltage = PACK_buf_to_float(data);
			esc->bus_current = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_TEMP_MOT_MOS1:
			esc->temp_motor = PACK_buf_to_float(data);
			esc->temp_mos1 = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_TEMP_MOS2_MOS3:
			esc->temp_mos2 = PACK_buf_to_float(data);
			esc->temp_mos3 = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_MOTOR_CURRENT:
			esc->Iq = PACK_buf_to_float(data);
			esc->Id = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_MOTOR_VOLTAGE:
			esc->Vq = PACK_buf_to_float(data);
			esc->Vd = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_STATUS:
			esc->status = PACK_buf_to_u32(data);
			break;
		case CAN_ID_FOC_HYPER:
			esc->cycles_fastloop = PACK_buf_to_u32(data);
			esc->cycles_hyperloop = PACK_buf_to_u32(data+4);
			break;
	}

}



void TASK_CAN_packet_cb(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len){
	TASK_CAN_node * node = TASK_CAN_get_node_from_id(sender);

	if(node != NULL && node->type == NODE_TYPE_ESC && node->data != NULL){
		esc_data * esc = node->data;
		TASK_CAN_packet_esc(esc, id, data, len);  //Process ESC data
		return;
	}

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

	if(has_even_parity(ret.angle) == parity_bit){
		ret.angle = 0;
		ret.error = MT6816_ERROR;
		ret.status = MT6816_PARITY_ERROR;
	}else if(SPIRxData & 0b10){  //Check no magnet bit
		ret.angle = 0;
		ret.error = MT6816_ERROR;
		ret.status = MT6816_NO_MAG;
	}

	return ret;
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

		//Use second source to verify accuracy
		if(error_state || axis_vars.error_latch){
		  //Check the PWM and the SPI agree and that the magnet present bit is low
			htim1.Instance->CCR1 = 0;
			htim15.Instance->CCR1 = 0;

			axis_vars.error_count++;
			if(axis_vars.encoder_error_limit && axis_vars.error_count > axis_vars.encoder_error_limit){
				axis_vars.error_latch = true;
				axis_vars.current_rel = 0.0f;
			}

		}
		else
		{
			htim1.Instance->CCR1 = (uint16_t)(axis_vars.ratioSPI*(float)htim1.Instance->ARR);
			htim15.Instance->CCR1 = (uint16_t)(2000.0f+2000.0f*axis_vars.ratioSPI);

			if(axis_vars.use_spi && axis_vars.use_pwm){
				axis_vars.current_rel = axis_vars.ratioSPI;
			}else if(axis_vars.use_pwm == true && axis_vars.use_spi == false){
				axis_vars.current_rel = axis_vars.ratioPWM;
			}else if(axis_vars.use_pwm == false && axis_vars.use_spi == true){
				axis_vars.current_rel = axis_vars.ratioSPI;
			}

		}


	    vTaskDelay(10);
	}
}


void TASK_CAN_telemetry_fast(TASK_CAN_handle * handle){
	TASK_CAN_add_float(handle	, CAN_ID_ADC1_2_REQ	  	, CAN_BROADCAST, axis_vars.current_rel	, 0.0f	, 0);
}

void TASK_CAN_telemetry_slow(TASK_CAN_handle * handle){

}

uint8_t CMD_esc_info(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	ttprintf("ESC detail info:\r\n");

	for(uint32_t i=0;i<NUM_NODES;i++){
		TASK_CAN_node * node = TASK_CAN_get_node_from_id(i);
		if(node!=NULL && node != NODE_OVERRUN){
			ttprintf("ID: %u\tType: %s\r\n", node->id, node->short_name);

			if(node->data && node->type == NODE_TYPE_ESC){
				esc_data * esc = node->data;
				ttprintf("\tSpeed: %f\r\n", esc->speed, esc->motor_current);
				ttprintf("\tIq: %f \tId: %f \tVq: %f Vd: %f\t\r\n", esc->Iq, esc->Id, esc->Vq, esc->Vd);
				ttprintf("\tADC1: %f \tADC2: %f\r\n", esc->adc1, esc->adc2);
				ttprintf("\tTemp Motor: %f \tTemp Mos1: %f \tTemp Mos2: %f \tTemp Mos3: %f\r\n", esc->temp_motor, esc->temp_mos1, esc->temp_mos2, esc->temp_mos3);
				ttprintf("\tBus voltage: %f \tBus current: %f\r\n", esc->bus_voltage, esc->bus_current);
				ttprintf("\tCycles fastloop: %u \tCycles hyperloop: %u\r\n", esc->cycles_fastloop, esc->cycles_hyperloop);

				ttprintf("\tMESC status: ");

				switch(esc->status){
				case MOTOR_STATE_INITIALISING:
					ttprintf("INITIALISING");
					break;
				case MOTOR_STATE_DETECTING:
					ttprintf("DETECTING");
					break;
				case MOTOR_STATE_MEASURING:
					ttprintf("MEASURING");
					break;
				case MOTOR_STATE_ALIGN:
					ttprintf("ALIGN");
					break;
				case MOTOR_STATE_OPEN_LOOP_STARTUP:
					ttprintf("OL STARTUP");
					break;
				case MOTOR_STATE_OPEN_LOOP_TRANSITION:
					ttprintf("OL TRANSITION");
					break;
				case MOTOR_STATE_TRACKING:
					ttprintf("TRACKING");
					break;
				case MOTOR_STATE_RUN:
					ttprintf("RUN");
					break;
				case MOTOR_STATE_GET_KV:
					ttprintf("GET KV");
					break;
				case MOTOR_STATE_TEST:
					ttprintf("TEST");
					break;
				case MOTOR_STATE_ERROR:
					ttprintf("ERROR");
					break;
				case MOTOR_STATE_RECOVERING:
					ttprintf("RECOVERING");
					break;
				case MOTOR_STATE_IDLE:
					ttprintf("IDLE");
					break;
				case MOTOR_STATE_SLAMBRAKE:
					ttprintf("SLAMBRAKE");
					break;
				}
				ttprintf("\r\n");

			}


		}
	}

	return TERM_CMD_EXIT_SUCCESS;

}


uint8_t CMD_clear_error(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	axis_vars.error_count = 0;
	axis_vars.error_latch = false;
	return TERM_CMD_EXIT_SUCCESS;
}

void MESCinterface_init(TERMINAL_HANDLE * handle){
	static bool is_init=false;
	if(is_init) return;
	is_init=true;

	axis_vars.check_pwm_vs_spi = true;
	axis_vars.use_pwm = true;
	axis_vars.use_spi = true;
	axis_vars.error_latch = false;
	axis_vars.error_count = 0;
	axis_vars.current_rel = 0.0f;

	if(CMD_varLoad(&null_handle, 0, NULL) == TERM_CMD_EXIT_ERROR){


	}

	TERM_addCommand(CMD_clear_error, "clear", "Clear errors", 0, &TERM_defaultList);

	TERM_addCommand(CMD_nodes, "nodes", "Node info", 0, &TERM_defaultList);
	TERM_addCommand(CMD_can_send, "can_send", "Send CAN message", 0, &TERM_defaultList);

	TERM_addCommand(CMD_esc_info, "esc", "ESC info", 0, &TERM_defaultList);

	REGISTER_apps(&TERM_defaultList);

	xTaskCreate(task_encoder, "tskEncoder", 128, NULL, osPriorityAboveNormal, NULL);


}
