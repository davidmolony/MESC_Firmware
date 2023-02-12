/*
 **
 ******************************************************************************
 * @file           : task_cli.c
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

#include "main.h"
#include "task_cli.h"
#include <string.h>
#include "cmsis_os.h"
#include "TTerm/Core/include/TTerm.h"
#include <stdio.h>
#include "stdarg.h"
#include "init.h"
#include "MESCinterface.h"

#include "usbd_cdc_if.h"

#define FLASH_SIZE 		130048

//uint8_t memo[2048];



uint32_t flash_clear(void * address, uint32_t len){
	FLASH_WaitForLastOperation(500);
	HAL_FLASH_Unlock();
	FLASH_WaitForLastOperation(500);
	eraseFlash((uint32_t)address, len);
	HAL_FLASH_Lock();
	FLASH_WaitForLastOperation(500);
	return len;
}

uint32_t flash_start_write(void * address, void * data, uint32_t len){
	uint8_t * buffer = data;
	FLASH_WaitForLastOperation(500);
	HAL_FLASH_Unlock();
	FLASH_WaitForLastOperation(500);
	uint32_t written=0;
	while(len){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)address, *buffer)==HAL_OK){
			written++;
		}
		buffer++;
		address++;
		len--;
	}
	return written;
}

uint32_t flash_write(void * address, void * data, uint32_t len){
	uint8_t * buffer = data;
	uint32_t written=0;
	while(len){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)address, *buffer)==HAL_OK){
			written++;
		}
		buffer++;
		address++;
		len--;
	}
	return written;
}

uint32_t flash_end_write(void * address, void * data, uint32_t len){
	uint8_t * buffer = data;
	uint32_t written=0;
	while(len){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)address, *buffer)==HAL_OK){
			written++;
		}
		buffer++;
		address++;
		len--;
	}
	FLASH_WaitForLastOperation(500);
	HAL_FLASH_Lock();
	FLASH_WaitForLastOperation(500);
	return written;
}


void putbuffer_uart(unsigned char *buf, unsigned int len, port_str * port){
	UART_HandleTypeDef *uart_handle = port->hw;


	if(port->half_duplex){
		uart_handle->Instance->CR1 &= ~USART_CR1_RE;
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(uart_handle, buf, len);
	vTaskDelay(1);
	while(uart_handle->gState == HAL_UART_STATE_BUSY_TX){
		vTaskDelay(1);
	}

	if(port->half_duplex) uart_handle->Instance->CR1 |= USART_CR1_RE;
	xSemaphoreGive(port->tx_semaphore);
}
volatile bool cmplt = false;

void USB_CDC_TransmitCplt(){
	cmplt = true;
}

void putbuffer_usb(unsigned char *buf, unsigned int len, port_str * port){
	xSemaphoreTake(port->tx_semaphore, portMAX_DELAY);
	while(CDC_Transmit_FS((uint8_t*)buf, len)== USBD_BUSY){
		vTaskDelay(1);
	}
	xSemaphoreGive(port->tx_semaphore);
}

void putbuffer(unsigned char *buf, unsigned int len, port_str * port){

	switch(port->hw_type){
	case HW_TYPE_UART:
		putbuffer_uart(buf, len, port);
		break;
	case HW_TYPE_USB:
		putbuffer_usb(buf, len, port);
		break;

	}
}

static void uart_init(port_str * port){
	UART_HandleTypeDef *uart_handle = port->hw;

	HAL_UART_MspInit(uart_handle);
	if(port->half_duplex){
		HAL_HalfDuplex_Init(uart_handle);
	}
	HAL_UART_Receive_DMA(uart_handle, port->rx_buffer, port->rx_buffer_size);
	CLEAR_BIT(uart_handle->Instance->CR3, USART_CR3_EIE);
}

static uint32_t uart_get_write_pos(port_str * port){

	UART_HandleTypeDef *uart_handle = port->hw;
	return ( ((uint32_t)port->rx_buffer_size - __HAL_DMA_GET_COUNTER(uart_handle->hdmarx)) & ((uint32_t)port->rx_buffer_size -1));
}


void ext_printf(port_str * port, const char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	char send_buffer[128];
	len = vsnprintf(send_buffer, 128, format, arg);
	va_end (arg);

	if(len > 0) {
		putbuffer((unsigned char*)send_buffer, len, port);
	}
}

StreamBufferHandle_t rx_stream;

void USB_CDC_Callback(uint8_t *buffer, uint32_t len){
	xStreamBufferSendFromISR(rx_stream, buffer, len, NULL);
}


void task_cli(void * argument)
{
	uint32_t rd_ptr=0;
	port_str * port = (port_str*) argument;
	uint8_t c;

	switch(port->hw_type){
		case HW_TYPE_UART:
			port->rx_buffer = pvPortMalloc(port->rx_buffer_size);
			uart_init(port);
			break;
		case HW_TYPE_USB:
			rx_stream = xStreamBufferCreate(port->rx_buffer_size, 1);
			break;
	}

	port->tx_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(port->tx_semaphore);

	port->term_block = xSemaphoreCreateBinary();
	xSemaphoreGive(port->term_block);

	TERMINAL_HANDLE * term_cli;

	switch(port->hw_type){
		case HW_TYPE_UART:
			term_cli =  TERM_createNewHandle(ext_printf, port, pdTRUE, &TERM_defaultList, NULL, "uart");
			break;
		case HW_TYPE_USB:
			term_cli =  TERM_createNewHandle(ext_printf, port, pdTRUE, &TERM_defaultList, NULL, "usb");
			break;
	}

	null_handle.varHandle = TERM_VAR_init(term_cli, (uint8_t*)getFlashBaseAddress(), FLASH_SIZE, flash_clear, flash_start_write, flash_write, flash_end_write);

	MESCinterface_init();

	//tcp_serv_init();

  /* Infinite loop */
	for(;;)
	{

		/* `#START TASK_LOOP_CODE` */
		switch(port->hw_type){
			case HW_TYPE_UART:
				while(rd_ptr != uart_get_write_pos(port)) {
					xSemaphoreTake(port->term_block, portMAX_DELAY);
					TERM_processBuffer(&port->rx_buffer[rd_ptr],1,term_cli);
					xSemaphoreGive(port->term_block);
					rd_ptr++;
					rd_ptr &= ((uint32_t)port->rx_buffer_size - 1);
				}
				break;
			case HW_TYPE_USB:
				while(xStreamBufferReceive(rx_stream, &c, 1, 1)){
					xSemaphoreTake(port->term_block, portMAX_DELAY);
					TERM_processBuffer(&c,1,term_cli);
					xSemaphoreGive(port->term_block);
				}

			break;
		}

		if(ulTaskNotifyTake(pdTRUE, 1)){
			HAL_UART_MspDeInit(port->hw);
			port->task_handle = NULL;
			vPortFree(port->rx_buffer);
			vTaskDelete(NULL);
			vTaskDelay(portMAX_DELAY);
		}

	}
}

void task_cli_init(port_str * port){
	if(port->task_handle == NULL){
		xTaskCreate(task_cli, "tskCLI", 1024, (void*)port, osPriorityNormal, &port->task_handle);
	}
}

void task_cli_kill(port_str * port){
	if(port->task_handle){
		xTaskNotify(port->task_handle, 0, eIncrement);
		vTaskDelay(200);
	}
}
