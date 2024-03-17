/*
 **
 ******************************************************************************
 * @file           : task_can.c
 * @brief          : CAN-BUS-Task for MESC and TTERM
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

#ifndef TASK_CAN_H_
#define TASK_CAN_H_

#include "main.h"
#include "task_cli.h"
#include "CAN_helper.h"

#ifdef HAL_CAN_MODULE_ENABLED


#define NUM_NODES 64

#define NODE_OVERRUN (TASK_CAN_node*) 0xFFFFFFFF

extern TASK_CAN_node nodes[NUM_NODES];

void TASK_CAN_init(port_str * port, char * short_name);
void TASK_CAN_set_stream(TASK_CAN_handle * handle, uint32_t id);
uint8_t CMD_nodes(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_can_send(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint32_t TASK_CAN_connect(TASK_CAN_handle * handle, uint16_t remote, uint8_t connect);

TASK_CAN_node * TASK_CAN_get_node_from_id(uint8_t id);


#endif
#endif
