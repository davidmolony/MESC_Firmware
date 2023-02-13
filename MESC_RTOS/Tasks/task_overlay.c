/*
 **
 ******************************************************************************
 * @file           : task_overlay.c
 * @brief          : Overlay realtime data in terminal
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


#include <stdlib.h>
#include <string.h>

#include "task_overlay.h"
#include "task_cli.h"
#include "cmsis_os.h"


/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


/* ------------------------------------------------------------------------ */
/*
 * Place user included headers, defines and task global data in the
 * below merge region section.
 */
/* `#START USER_INCLUDE SECTION` */
#include "TTerm/Core/include/TTerm.h"

#include "MESCmotor_state.h"

/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * User defined task-local code that is used to process commands, control
 * operations, and/or generrally do stuff to make the taks do something
 * meaningful.
 */
/* `#START USER_TASK_LOCAL_CODE` */


#define OVERLAY_OUTPUT_NONE			0
#define OVERLAY_OUTPUT_VT100 		1
#define OVERLAY_OUTPUT_CSV 			2


void show_overlay(TERMINAL_HANDLE * handle){

	MESC_motor_typedef * motor_curr = &mtr[0];

	TERM_sendVT100Code(handle, _VT100_CURSOR_SAVE_POSITION,0);
	TERM_sendVT100Code(handle, _VT100_CURSOR_DISABLE,0);

	uint8_t row_pos = 1;
	uint8_t col_pos = 90;
	TERM_Box(handle, row_pos, col_pos, row_pos + 6, col_pos + 31);
	TERM_setCursorPos(handle, row_pos + 1, col_pos + 1);
	ttprintf("Bus Voltage:       %10.1fV", motor_curr->Conv.Vbus);

	TERM_setCursorPos(handle, row_pos + 2, col_pos + 1);
	ttprintf("Bus Current:      %10.2f A", motor_curr->FOC.Ibus);

	TERM_setCursorPos(handle, row_pos + 3, col_pos + 1);
	ttprintf("Speed:         %10.2f ERPM", motor_curr->FOC.eHz*60.0);

	TERM_setCursorPos(handle, row_pos + 4, col_pos + 1);
	ttprintf("Power:            %10.2f W", motor_curr->FOC.currentPower.q);

	TERM_setCursorPos(handle, row_pos + 5, col_pos + 1);
	ttprintf("MESC status: ");

	switch(motor_curr->MotorState){
	case MOTOR_STATE_INITIALISING:
		ttprintf("     INITIALISING");
		break;
	case MOTOR_STATE_DETECTING:
		ttprintf("        DETECTING");
		break;
	case MOTOR_STATE_MEASURING:
		ttprintf("        MEASURING");
		break;
	case MOTOR_STATE_ALIGN:
		ttprintf("            ALIGN");
		break;
	case MOTOR_STATE_OPEN_LOOP_STARTUP:
		ttprintf("       OL STARTUP");
		break;
	case MOTOR_STATE_OPEN_LOOP_TRANSITION:
		ttprintf("    OL TRANSITION");
		break;
	case MOTOR_STATE_TRACKING:
		ttprintf("         TRACKING");
		break;
	case MOTOR_STATE_RUN:
		ttprintf("              RUN");
		break;
	case MOTOR_STATE_GET_KV:
		ttprintf("           GET KV");
		break;
	case MOTOR_STATE_TEST:
		ttprintf("             TEST");
		break;
	case MOTOR_STATE_ERROR:
		ttprintf("            ERROR");
		break;
	case MOTOR_STATE_RECOVERING:
		ttprintf("       RECOVERING");
		break;
	case MOTOR_STATE_IDLE:
		ttprintf("             IDLE");
		break;
	case MOTOR_STATE_SLAMBRAKE:
		ttprintf("        SLAMBRAKE");
		break;
	}


	TERM_sendVT100Code(handle, _VT100_CURSOR_RESTORE_POSITION,0);
	TERM_sendVT100Code(handle, _VT100_CURSOR_ENABLE,0);

}

void show_overlay_csv(TERMINAL_HANDLE * handle){

	uint32_t currPos = 0;
	TermVariableDescriptor * head = handle->varHandle->varListHead;
	TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;
    for(;currPos < head->nameLength; currPos++){

    	if(currVar->flags & FLAG_TELEMETRY_ON){
    		print_var_helperfunc(handle, currVar, 2);
    	}
    	currVar = currVar->nextVar;
    }


}

/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * This is the main procedure that comprises the task.  Place the code required
 * to preform the desired function within the merge regions of the task procedure
 * to add functionality to the task.
 */
void task_overlay_TaskProc(void *pvParameters) {
	/*
	 * Add and initialize local variables that are allocated on the Task stack
	 * the the section below.
	 */
	/* `#START TASK_VARIABLES` */

    TERMINAL_HANDLE * handle = pvParameters;

    port_str * port = handle->port;

	/* `#END` */

	/*
	 * Add the task initialzation code in the below merge region to be included
	 * in the task.
	 */
	/* `#START TASK_INIT_CODE` */

	/* `#END` */


    for (;;) {
		/* `#START TASK_LOOP_CODE` */
        xSemaphoreTake(port->term_block, portMAX_DELAY);

        switch(port->overlay_handle.output_type){
			case OVERLAY_OUTPUT_VT100:
				show_overlay(handle);
				break;
			case OVERLAY_OUTPUT_CSV:
				show_overlay_csv(handle);
				break;
        }

        xSemaphoreGive(port->term_block);

        vTaskDelay(500 / portTICK_PERIOD_MS);

	}
}

/*****************************************************************************
* Helper function for spawning the overlay task
******************************************************************************/
void start_overlay_task(TERMINAL_HANDLE * handle){

	port_str * port = handle->port;

    if (port->overlay_handle.task_handle == NULL) {

        xTaskCreate(task_overlay_TaskProc, "Overlay", 512, handle, osPriorityNormal, &port->overlay_handle.task_handle);

    }
}


/*****************************************************************************
* Helper function for killing the overlay task
******************************************************************************/
void stop_overlay_task(TERMINAL_HANDLE * handle){
	port_str * port = handle->port;
    if (port->overlay_handle.task_handle != NULL) {
        vTaskDelete(port->overlay_handle.task_handle);
    	port->overlay_handle.task_handle = NULL;
    }
}


/*****************************************************************************
*
******************************************************************************/
uint8_t CMD_status(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf("Usage: status [start|stop]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }

    port_str * port = handle->port;

	if(strcmp(args[0], "start") == 0){
		port->overlay_handle.output_type = OVERLAY_OUTPUT_VT100;
		start_overlay_task(handle);
        return TERM_CMD_EXIT_SUCCESS;
	}
	if(strcmp(args[0], "csv") == 0){
		port->overlay_handle.output_type = OVERLAY_OUTPUT_CSV;
		start_overlay_task(handle);
		return TERM_CMD_EXIT_SUCCESS;
	}
	if(strcmp(args[0], "stop") == 0){
		port->overlay_handle.output_type = OVERLAY_OUTPUT_NONE;
		stop_overlay_task(handle);
        return TERM_CMD_EXIT_SUCCESS;
	}
    return TERM_CMD_EXIT_SUCCESS;
}


void log_mod(TERMINAL_HANDLE * handle, char * name, bool delete){
	uint32_t currPos = 0;
	TermVariableDescriptor * head = handle->varHandle->varListHead;
	TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;

	for(;currPos < head->nameLength; currPos++){

		if(strcmp(name, currVar->name)==0){
			if(delete){
				ttprintf("Removed [%s] from log list\r\n", name);
				TERM_clearFlag(currVar, FLAG_TELEMETRY_ON);
			}else{
				ttprintf("Added [%s] to log list\r\n", name);
				TERM_setFlag(currVar, FLAG_TELEMETRY_ON);
			}
		}
		currVar = currVar->nextVar;
	}
}



/*****************************************************************************
*
******************************************************************************/
uint8_t CMD_log(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf("Usage: log\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }

    bool flag_list = false;
    bool flag_reset = false;

	for(int i=0;i<argCount;i++){
		if(strcmp(args[i], "-l")==0){
			flag_list = true;
		}
		if(strcmp(args[i], "-r")==0){
			flag_reset = true;
		}
		if(strcmp(args[i], "-a")==0){
			if(i+1 < argCount){
				log_mod(handle, args[i+1], false);
			}
		}
		if(strcmp(args[i], "-d")==0){
			if(i+1 < argCount){
				log_mod(handle, args[i+1], true);
			}
		}
		if(strcmp(args[i], "-?")==0){
			ttprintf("Usage: log [flags]\r\n");
			return TERM_CMD_EXIT_SUCCESS;
		}
	}

	if(flag_list){
		uint32_t currPos = 0;
		uint32_t count=0;
		TermVariableDescriptor * head = handle->varHandle->varListHead;
		TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;


		ttprintf("Datasources selected for log output:\r\n");

	    for(;currPos < head->nameLength; currPos++){

	    	if(currVar->flags & FLAG_TELEMETRY_ON){
	    		ttprintf("\t%u: %s\r\n", count, currVar->name);
	    		count++;
	    	}
	    	currVar = currVar->nextVar;
	    }
	    ttprintf("EOL\r\n");
	}

	if(flag_reset){
		uint32_t currPos = 0;
		TermVariableDescriptor * head = handle->varHandle->varListHead;
		TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;

		for(;currPos < head->nameLength; currPos++){

			if(currVar->flags & FLAG_TELEMETRY_ON){
				TERM_clearFlag(currVar, FLAG_TELEMETRY_ON);
			}
			currVar = currVar->nextVar;
		}
		ttprintf("Log list reset...\r\n");
	}


    return TERM_CMD_EXIT_SUCCESS;
}



