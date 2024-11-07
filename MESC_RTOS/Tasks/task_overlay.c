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
#include "task_can.h"
#include "cmsis_os.h"
#include <stdio.h>



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
#ifdef MESC
#include "MESCfoc.h"
#include "MESCmotor_state.h"
#endif

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
#define OVERLAY_OUTPUT_JSON			3


void show_overlay(TERMINAL_HANDLE * handle){
#ifdef MESC
	MESC_motor_typedef * motor_curr = &mtr[0];

	TERM_sendVT100Code(handle, _VT100_CURSOR_SAVE_POSITION,0);
	TERM_sendVT100Code(handle, _VT100_CURSOR_DISABLE,0);

	uint8_t row_pos = 1;
	uint8_t col_pos = 90;
	TERM_Box(handle, row_pos, col_pos, row_pos + 6, col_pos + 31);
	TERM_setCursorPos(handle, row_pos + 1, col_pos + 1);
	ttprintf("Bus Voltage:       %10.1fV", (double)motor_curr->Conv.Vbus);

	TERM_setCursorPos(handle, row_pos + 2, col_pos + 1);
	ttprintf("Bus Current:      %10.2f A", (double)motor_curr->FOC.Ibus);

	TERM_setCursorPos(handle, row_pos + 3, col_pos + 1);
	ttprintf("Speed:         %10.2f ERPM", (double)(motor_curr->FOC.eHz*60.0f));

	TERM_setCursorPos(handle, row_pos + 4, col_pos + 1);
	ttprintf("Power:            %10.2f W", (double)motor_curr->FOC.currentPower.q);

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
	case MOTOR_STATE_RUN_BLDC:
		ttprintf("         RUN BLDC");
		break;
	}


	TERM_sendVT100Code(handle, _VT100_CURSOR_RESTORE_POSITION,0);
	TERM_sendVT100Code(handle, _VT100_CURSOR_ENABLE,0);
#endif

}

uint32_t format_json(TERMINAL_HANDLE * handle, TermVariableDescriptor * desc, char * buffer, int32_t len){
	uint32_t written=0;
	uint32_t bytes_written=0;
	if(desc->type == TERM_VARIABLE_CHAR || desc->type == TERM_VARIABLE_STRING){
		written = snprintf(buffer, len, "\"%s\":\"", desc->name);
	}else{
		written = snprintf(buffer, len, "\"%s\":", desc->name);
	}
	buffer += written;
	len -= written;
	bytes_written += written;
	written = TERM_var2str(handle, desc, buffer, len);
	buffer += written;
	len -= written;
	bytes_written += written;
	if(desc->type == TERM_VARIABLE_CHAR || desc->type == TERM_VARIABLE_STRING){
		written = snprintf(buffer, len, "\",");
	}else{
		written = snprintf(buffer, len, ",");
	}
	len -= written;
	bytes_written += written;
	//}

	return bytes_written;
}

void show_overlay_json(TERMINAL_HANDLE * handle){

	char buffer[512];
	char * ptr = buffer;
	uint32_t bytes_left = sizeof(buffer);
	int32_t written;

	bool found = false;

	*ptr = '{';
	ptr++;
	bytes_left--;

	uint32_t currPos = 0;
	TermVariableDescriptor * head = handle->varHandle->varListHead;
	TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;
    for(;currPos < head->nameLength; currPos++){

    	if(currVar->flags & FLAG_TELEMETRY_ON){
    		written = format_json(handle, currVar, ptr, bytes_left);
    		bytes_left -= written;
    		ptr += written;
    		found = true;

    	}
    	currVar = currVar->nextVar;
    }
    if(found) ptr--; //remove comma

    written = snprintf(ptr,bytes_left,"}\r\n");
    bytes_left -= written;

    ttprintf(NULL, buffer, sizeof(buffer)-bytes_left);

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
			case OVERLAY_OUTPUT_JSON:
				show_overlay_json(handle);
				break;
        }

        xSemaphoreGive(port->term_block);

        vTaskDelay(pdMS_TO_TICKS(port->overlay_handle.delay));

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
		port->overlay_handle.delay = 500;  //ms
		port->overlay_handle.output_type = OVERLAY_OUTPUT_VT100;
		start_overlay_task(handle);
        return TERM_CMD_EXIT_SUCCESS;
	}
	if(strcmp(args[0], "json") == 0){
		port->overlay_handle.delay = 100;  //ms
		port->overlay_handle.output_type = OVERLAY_OUTPUT_JSON;
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

void print_array(TERMINAL_HANDLE * handle, char * name, void * array, uint32_t count, uint32_t start_pos, uint32_t type_size ,TermVariableType type){

	char buffer[512];
	char * ptr = buffer;
	uint32_t bytes_left = sizeof(buffer);
	int32_t written;

	uint32_t u_var = 0;
	int32_t i_var = 0;


	uint32_t currPos = start_pos;

	written = snprintf(ptr, bytes_left, "\"%s\":[", name);
	ptr += written;
	bytes_left -= written;



	for(uint32_t i=0;i<count;i++){

		switch(type){
			case TERM_VARIABLE_FLOAT_ARRAY:
				written = snprintf(ptr, bytes_left, "%.2f,", (double)((float*)array)[currPos]);
				break;
			case TERM_VARIABLE_UINT_ARRAY:
				switch(type_size){
					case 1:
						u_var = ((uint8_t*)array)[currPos];
						break;
					case 2:
						u_var = ((uint16_t*)array)[currPos];
						break;
					case 4:
						u_var = ((uint32_t*)array)[currPos];
						break;
				}

				written = snprintf(ptr, bytes_left, "%lu,", u_var);
				break;
			case TERM_VARIABLE_INT_ARRAY:
					switch(type_size){
						case 1:
							i_var = ((uint8_t*)array)[currPos];
							break;
						case 2:
							i_var = ((uint16_t*)array)[currPos];
							break;
						case 4:
							i_var = ((uint32_t*)array)[currPos];
							break;
					}

					written = snprintf(ptr, bytes_left, "%li,", i_var);
				break;
			default:
				break;
		}

		currPos++;
		if(currPos == count){
			currPos=0;
		}

		ptr += written;
		bytes_left -= written;
		if(i == count-1){
			*(ptr-1) = ']';	//remove trailing comma
		}

		if(bytes_left < 16 || i == count-1){

			ttprintf(NULL, buffer, sizeof(buffer)-bytes_left);
			ptr = buffer;
			bytes_left = sizeof(buffer);
		}

	}

}

void print_index(TERMINAL_HANDLE * handle, char * name, uint32_t count, float increment){

	char buffer[512];
	char * ptr = buffer;
	uint32_t bytes_left = sizeof(buffer);
	int32_t written;

	float index=0.0f;

	written = snprintf(ptr, bytes_left, "\"%s\":[", name);
	ptr += written;
	bytes_left -= written;

	for(uint32_t i=0;i<count;i++){

		written = snprintf(ptr, bytes_left, "%f,", (double)index);
		index += increment;

		ptr += written;
		bytes_left -= written;
		if(i == count-1){
			*(ptr-1) = ']';	//remove trailing comma
		}

		if(bytes_left < 16 || i == count-1){

			ttprintf(NULL, buffer, sizeof(buffer)-bytes_left);
			ptr = buffer;
			bytes_left = sizeof(buffer);
		}

	}

}

void log_fastloop(TERMINAL_HANDLE * handle){

#ifdef MESC
	MESC_motor_typedef * motor_curr = &mtr[0];

	motor_curr->logging.lognow = 1;

	vTaskDelay(100);


	motor_curr->logging.print_samples_now = 0;
	motor_curr->logging.lognow = 0;
	vTaskDelay(10);


	int current_sample_pos = motor_curr->logging.current_sample;

	ttprintf("\r\n{");
	print_index(handle, "time", LOGLENGTH, mtr[0].FOC.pwm_period);
	ttprintf(",");
	print_array(handle, "Vbus.V.y1", motor_curr->logging.Vbus, LOGLENGTH, current_sample_pos, sizeof(float), TERM_VARIABLE_FLOAT_ARRAY);
	ttprintf(",");
	print_array(handle, "Iu.I_phase.y1", motor_curr->logging.Iu, LOGLENGTH, current_sample_pos, sizeof(float), TERM_VARIABLE_FLOAT_ARRAY);
	ttprintf(",");
	print_array(handle, "Iv.I_phase.y1", motor_curr->logging.Iv, LOGLENGTH, current_sample_pos, sizeof(float), TERM_VARIABLE_FLOAT_ARRAY);
	ttprintf(",");
	print_array(handle, "Iw.I_phase.y1", motor_curr->logging.Iw, LOGLENGTH, current_sample_pos, sizeof(float), TERM_VARIABLE_FLOAT_ARRAY);
	ttprintf(",");
	print_array(handle, "Vd.V_dq.y1", motor_curr->logging.Vd, LOGLENGTH, current_sample_pos, sizeof(float), TERM_VARIABLE_FLOAT_ARRAY);
	ttprintf(",");
	print_array(handle, "Vq.V_dq.y1", motor_curr->logging.Vq, LOGLENGTH, current_sample_pos, sizeof(float), TERM_VARIABLE_FLOAT_ARRAY);
	ttprintf(",");
	print_array(handle, "angle.misc.y1", motor_curr->logging.angle, LOGLENGTH, current_sample_pos, sizeof(uint16_t), TERM_VARIABLE_UINT_ARRAY);
	ttprintf("}\r\n");
	print_array(handle, "hall.misc.y1", motor_curr->logging.hallstate, LOGLENGTH, current_sample_pos, sizeof(uint16_t), TERM_VARIABLE_UINT_ARRAY);
	ttprintf("}\r\n");

	motor_curr->logging.lognow = 1;
#endif
}


/*****************************************************************************
*
******************************************************************************/
uint8_t CMD_log(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

    bool flag_list = false;
    bool flag_reset = false;
    bool flag_fastloop = false;

    port_str * port = handle->port;

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
		if(strcmp(args[i], "-s")==0){
			if(i+1 < argCount){
				port->overlay_handle.delay = strtoul(args[i+1], NULL, 10);
			}
		}
		if(strcmp(args[i], "-?")==0){
			ttprintf("Usage: log [flags]\r\n");
			ttprintf("\t -a\t Add var to log\r\n");
			ttprintf("\t -d\t Delete var from log\r\n");
			ttprintf("\t -r\t Delete all vars from log\r\n");
			ttprintf("\t -l\t List vars\r\n");
			ttprintf("\t -s\t Log speed [ms]\r\n");
			return TERM_CMD_EXIT_SUCCESS;
		}
		if(strcmp(args[i], "-fl")==0){
			flag_fastloop = true;
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

	if(flag_fastloop){
	#ifdef LOGGING
		log_fastloop(handle);
	#else
		ttprintf("Fastloop logging not enabled in firmware\r\n");
	#endif
	}


    return TERM_CMD_EXIT_SUCCESS;
}



