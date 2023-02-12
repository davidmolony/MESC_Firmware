/*
 **
 ******************************************************************************
 * @file           : init.c
 * @brief          : Bring up the system
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

#include "init.h"
#include "task_cli.h"
#include "main.h"
#include "usbd_def.h"
#include "stdarg.h"

extern UART_HandleTypeDef HW_UART;
extern USBD_HandleTypeDef hUsbDeviceFS;

port_str main_uart = {	.hw = &HW_UART,
						.hw_type = HW_TYPE_UART,
					    .rx_buffer_size = 512,
						.half_duplex = false,
						.task_handle = NULL
};

port_str main_usb = {	.hw = &hUsbDeviceFS,
						.hw_type = HW_TYPE_USB,
					    .rx_buffer_size = 512,
						.half_duplex = false,
						.task_handle = NULL
};



#if EXTENDED_PRINTF == 1
uint32_t null_printf(void * port, const char* format, ...) {
	va_list arg;
	va_start (arg, format);
	va_end (arg);
	return 0;
}
#else
uint32_t null_printf(const char* format, ...) {
	va_list arg;
	va_start (arg, format);
	va_end (arg);
	return 0;
}
#endif


TERMINAL_HANDLE null_handle = {
		.print=null_printf
};


void init_system(void){

	task_cli_init(&main_usb);
	task_cli_init(&main_uart);
}
