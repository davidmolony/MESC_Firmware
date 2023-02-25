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

#include "main.h"
#include "init.h"
#include "TTerm/Core/include/TTerm.h"
#include "task_cli.h"
#include "task_overlay.h"
#include "DASHinterface.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>


#include "fatfs.h"
#include "tcp_serv.h"


float test;
void populate_vars(){
	//		   | Variable							| MIN		| MAX		| NAME			| DESCRIPTION							| RW			| CALLBACK	| VAR LIST HANDLE
	TERM_addVar(can1.node_id						, 0		, 255	, "node_id"		, "Node ID"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);


}


void TASK_CAN_packet_cb(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len){
	switch(id){
		case CAN_ID_IQREQ:{
			float req = buffer_to_float(data);

			break;
		}
		case CAN_ID_CONNECT:{

			break;
		}
		default:
			break;
	}
}



void DASHinterface_init(TERMINAL_HANDLE * handle){
	static bool is_init=false;
	if(is_init) return;
	is_init=true;


	populate_vars();


	TERM_addCommand(CMD_status, "status", "Realtime data", 0, &TERM_defaultList);
	TERM_addCommand(CMD_ifconfig, "ifconfig", "ifconfig", 0, &TERM_defaultList);

	TERM_addCommand(CMD_nodes, "nodes", "Node info", 0, &TERM_defaultList);
	TERM_addCommand(CMD_can_send, "can_send", "Send CAN message", 0, &TERM_defaultList);

	TermCommandDescriptor * varAC = TERM_addCommand(CMD_log, "log", "Configure logging", 0, &TERM_defaultList);
	TERM_addCommandAC(varAC, TERM_varCompleter, null_handle.varHandle->varListHead);

	REGISTER_apps(&TERM_defaultList);


	tcp_serv_init();



}
