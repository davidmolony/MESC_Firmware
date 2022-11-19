/*
 * MESC
 *
 * Copyright (c) 2022 Jens Kerrinnes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


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


void show_overlay(TERMINAL_HANDLE * handle){

	TERM_sendVT100Code(handle, _VT100_CURSOR_SAVE_POSITION,0);
	TERM_sendVT100Code(handle, _VT100_CURSOR_DISABLE,0);

	uint8_t row_pos = 1;
	uint8_t col_pos = 90;
	TERM_Box(handle, row_pos, col_pos, row_pos + 6, col_pos + 31);
	TERM_setCursorPos(handle, row_pos + 1, col_pos + 1);
	ttprintf("Bus Voltage:       %10.1fV", measurement_buffers.ConvertedADC[0][1]);

	TERM_setCursorPos(handle, row_pos + 2, col_pos + 1);
	ttprintf("Bus Current:      %10.2f A", foc_vars.Ibus);

	TERM_setCursorPos(handle, row_pos + 3, col_pos + 1);
	ttprintf("Speed:         %10.2f ERPM", foc_vars.eHz*60.0);

	TERM_setCursorPos(handle, row_pos + 4, col_pos + 1);
	ttprintf("Power:            %10.2f W", foc_vars.currentPower.q);

	TERM_setCursorPos(handle, row_pos + 5, col_pos + 1);
	ttprintf("MESC status: ");

	switch(MotorState){
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
	}


	TERM_sendVT100Code(handle, _VT100_CURSOR_RESTORE_POSITION,0);
	TERM_sendVT100Code(handle, _VT100_CURSOR_ENABLE,0);

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
        show_overlay(handle);

        xSemaphoreGive(port->term_block);

        vTaskDelay(500 / portTICK_PERIOD_MS);

	}
}

/*****************************************************************************
* Helper function for spawning the overlay task
******************************************************************************/
void start_overlay_task(TERMINAL_HANDLE * handle){

	port_str * port = handle->port;

    if (port->overlay_handle == NULL) {

        xTaskCreate(task_overlay_TaskProc, "Overlay", 512, handle, osPriorityNormal, &port->overlay_handle);

    }
}


/*****************************************************************************
* Helper function for killing the overlay task
******************************************************************************/
void stop_overlay_task(TERMINAL_HANDLE * handle){
	port_str * port = handle->port;
    if (port->overlay_handle != NULL) {
        vTaskDelete(port->overlay_handle);
    	port->overlay_handle = NULL;
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
	if(strcmp(args[0], "start") == 0){
		start_overlay_task(handle);
        return TERM_CMD_EXIT_SUCCESS;
	}
	if(strcmp(args[0], "stop") == 0){
		stop_overlay_task(handle);
        return TERM_CMD_EXIT_SUCCESS;
	}
    return TERM_CMD_EXIT_SUCCESS;
}

