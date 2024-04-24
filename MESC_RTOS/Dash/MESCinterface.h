/*
 **
 ******************************************************************************
 * @file           : MESCinterface.h
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


#ifndef INC_MESC_INTERFACE_H_
#define INC_MESC_INTERFACE_H_

#include "Tasks/task_cli.h"
#include "Tasks/task_can.h"


#define CAN_NAME "DASHF407"


typedef struct{
	uint8_t id_speed;
	float speed;
	uint8_t id_voltage;
	float bus_voltage;
} dash_data;



void MESCinterface_init(TERMINAL_HANDLE * handle);


#endif /* INC_MESC_INTERFACE_H_ */
