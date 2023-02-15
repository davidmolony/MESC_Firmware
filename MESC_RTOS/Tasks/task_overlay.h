/*
 **
 ******************************************************************************
 * @file           : task_overlay.h
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

#if !defined(overlay_TASK_H)
#define overlay_TASK_H


#include "TTerm/Core/include/TTerm.h"


typedef struct{
	TaskHandle_t task_handle;
	uint8_t output_type;
	uint16_t delay;
}overlay_handle;

/*
 * Add user task definitions, types, includes and other things in the below
 * merge region to customize the task.
 */
/* `#START USER_TYPES_AND_DEFINES` */

/* `#END` */

void tsk_overlay_TaskProc(void *pvParameters);

void start_overlay_task(TERMINAL_HANDLE * handle);
void stop_overlay_task(TERMINAL_HANDLE * handle);

uint8_t CMD_status(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_log(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);


/*
 * Add user function prototypes in the below merge region to add user
 * functionality to the task definition.
 */
/* `#START USER_TASK_PROTOS` */

/* `#END` */

/* ------------------------------------------------------------------------ */
#endif
/* [] END OF FILE */
