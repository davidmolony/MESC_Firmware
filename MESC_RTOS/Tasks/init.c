/*
 * init.c
 *
 *  Created on: Jul 9, 2022
 *      Author: jensk
 */

#include "task_cli.h"
#include "main.h"
#include "usbd_def.h"
#include "stdarg.h"
#include "TTerm/Core/include/TTerm.h"

extern UART_HandleTypeDef huart1;
extern USBD_HandleTypeDef hUsbDeviceFS;

port_str main_uart = {	.hw = &huart1,
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
