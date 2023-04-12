/*
 **
 ******************************************************************************
 * @file           : task_cli.h
 * @brief          : IO-Task for TTerm
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

#ifndef TASK_CLI_H_
#define TASK_CLI_H_

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "main.h"
#include "task.h"
#include "stdbool.h"
#include "semphr.h"

#include "task_overlay.h"


void cli_start_console();

#define HW_TYPE_NULL 	0
#define HW_TYPE_UART 	1
#define HW_TYPE_USB 	2
#define HW_TYPE_CAN 	3



typedef struct{
	void * hw;
	uint8_t hw_type;
	uint8_t * rx_buffer;
	uint16_t rx_buffer_size;  //power of 2
	bool half_duplex;
	TaskHandle_t task_handle;
	overlay_handle overlay_handle;
	SemaphoreHandle_t term_block;
	SemaphoreHandle_t tx_semaphore;
	StreamBufferHandle_t rx_stream;
	StreamBufferHandle_t tx_stream;
} port_str;


void task_cli_init(port_str * port);
void task_cli_kill(port_str * port);

void putbuffer_can(unsigned char *buf, unsigned int len, port_str * port);

#endif /* TASK_LED_H_ */

