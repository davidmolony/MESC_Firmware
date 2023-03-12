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

#include "MESCinterface.h"
#include "main.h"
#include "Tasks/init.h"
#include "TTerm/Core/include/TTerm.h"
#include "Tasks/task_cli.h"
#include "Tasks/task_can.h"
#include "Tasks/task_overlay.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>


#include "fatfs.h"
#include "tcp_serv.h"

#include "ssd1306.h"
#include "ssd1306_tests.h"

#include <stdio.h>


dash_data dash;

void * TASK_CAN_allocate_node(TASK_CAN_handle * handle, TASK_CAN_node * node){
	if(memcmp(node->short_name, "ESC", 3)){
		memset(node, 0, sizeof(TASK_CAN_node));
		node->type = NODE_TYPE_ESC;
		node->data = pvPortMalloc(sizeof(esc_data));
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
	TERM_addVar(dash.id_speed						, 1			, 254		, "id_speed"	, "Obtain speed from ID"				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(dash.id_voltage						, 1			, 254		, "id_voltage"	, "Obtain bus voltage from ID"			, VAR_ACCESS_RW	, NULL		, &TERM_varList);

}

float req = 0.0;


void TASK_CAN_packet_cb(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len){
	switch(id){
		case CAN_ID_SPEED:{
			TASK_CAN_node * node = TASK_CAN_get_node_from_id(sender);
			if(node != NULL && node->type == NODE_TYPE_ESC && node->data != NULL){
				esc_data * esc = node->data;
				esc->speed = buffer_to_float(data);
			}
			break;
		}
		case CAN_ID_BUS_VOLTAGE:{
			TASK_CAN_node * node = TASK_CAN_get_node_from_id(sender);
			if(node != NULL && node->type == NODE_TYPE_ESC && node->data != NULL){
				esc_data * esc = node->data;
				esc->bus_voltage = buffer_to_float(data);
			}
			break;
		}
		case CAN_ID_BUS_CURRENT:{
			TASK_CAN_node * node = TASK_CAN_get_node_from_id(sender);
			if(node != NULL && node->type == NODE_TYPE_ESC && node->data != NULL){
				esc_data * esc = node->data;
				esc->bus_current = buffer_to_float(data);
			}
			break;
		}
		case CAN_ID_TEMP_MOT_MOS1:{
			TASK_CAN_node * node = TASK_CAN_get_node_from_id(sender);
			if(node != NULL && node->type == NODE_TYPE_ESC && node->data != NULL){
				esc_data * esc = node->data;
				esc->temp_motor = buffer_to_float(data);
				esc->temp_mos1 = buffer_to_float(data+4);
			}
			break;
		}
		case CAN_ID_TEMP_MOS2_MOS3:{
			TASK_CAN_node * node = TASK_CAN_get_node_from_id(sender);
			if(node != NULL && node->type == NODE_TYPE_ESC && node->data != NULL){
				esc_data * esc = node->data;
				esc->temp_mos2 = buffer_to_float(data);
				esc->temp_mos3 = buffer_to_float(data+4);
			}
			break;
		}
		case CAN_ID_MOTOR_CURRENT:{
			TASK_CAN_node * node = TASK_CAN_get_node_from_id(sender);
			if(node != NULL && node->type == NODE_TYPE_ESC && node->data != NULL){
				esc_data * esc = node->data;
				esc->motor_current = buffer_to_float(data);
			}
			break;
		}
		case CAN_ID_STATUS:{
			TASK_CAN_node * node = TASK_CAN_get_node_from_id(sender);
			if(node != NULL && node->type == NODE_TYPE_ESC && node->data != NULL){
				esc_data * esc = node->data;
				esc->status = buffer_to_uint32(data);
			}
			break;
		}
		default:
			break;
	}
}


void task_ssd(void * argument){
	ssd1306_Init();
	char buffer[64];


	while(1){
		uint32_t y = 0;

	    ssd1306_Fill(Black);

	    ssd1306_SetCursor(2, y);
	    snprintf(buffer,sizeof(buffer),"Speed: %2.2f", 0);
	    ssd1306_WriteString(buffer, Font_11x18, White);
	    y += 19;

	    ssd1306_SetCursor(2, y);
	    snprintf(buffer,sizeof(buffer),"Curr : %2.2f", 0);
	    ssd1306_WriteString(buffer, Font_11x18, White);
	    y += 19;

	    ssd1306_SetCursor(2, y);
	    snprintf(buffer,sizeof(buffer),"Volt : %2.2f", 0);
		ssd1306_WriteString(buffer, Font_11x18, White);
		y += 19;


	    ssd1306_UpdateScreen();
	    vTaskDelay(50);

	}
}

void TASK_CAN_telemetry_fast(TASK_CAN_handle * handle){

}

void TASK_CAN_telemetry_slow(TASK_CAN_handle * handle){

}


void MESCinterface_init(TERMINAL_HANDLE * handle){
	static bool is_init=false;
	if(is_init) return;
	is_init=true;


	populate_vars();

	if(CMD_varLoad(&null_handle, 0, NULL) == TERM_CMD_EXIT_ERROR){


	}

	TERM_addCommand(CMD_status, "status", "Realtime data", 0, &TERM_defaultList);
	TERM_addCommand(CMD_ifconfig, "ifconfig", "ifconfig", 0, &TERM_defaultList);

	TERM_addCommand(CMD_nodes, "nodes", "Node info", 0, &TERM_defaultList);
	TERM_addCommand(CMD_can_send, "can_send", "Send CAN message", 0, &TERM_defaultList);

	TermCommandDescriptor * varAC = TERM_addCommand(CMD_log, "log", "Configure logging", 0, &TERM_defaultList);
	TERM_addCommandAC(varAC, TERM_varCompleter, null_handle.varHandle->varListHead);

	REGISTER_apps(&TERM_defaultList);


	tcp_serv_init();



	vTaskDelay(200);
	f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);



	vTaskDelay(100);
;
	xTaskCreate(task_ssd, "tskSSD", 1024, NULL, osPriorityNormal, NULL);



}
