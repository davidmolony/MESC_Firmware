/*
 * init.c
 *
 *  Created on: Jul 9, 2022
 *      Author: jensk
 */

#include "task_cli.h"
#include "main.h"
#include "usbd_def.h"

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

void init_system(void){

	task_cli_init(&main_usb);
	task_cli_init(&main_uart);
}

