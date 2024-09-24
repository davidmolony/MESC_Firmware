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

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#ifdef DASH
#include "DASH/MESCinterface.h"
#endif
#ifdef MESC
#include "MESC/MESCinterface.h"
#endif
#ifdef AXIS
#include "AXIS/MESCinterface.h"
#endif

#include "Common/RTOS_flash.h"
#include "main.h"
#include "task_cli.h"
#include "cmsis_os.h"
#include "TTerm/Core/include/TTerm.h"
#include "task_can.h"

#ifdef MESC_UART_USB
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#endif


void putbuffer_uart(unsigned char *buf, unsigned int len, port_str * port){
	UART_HandleTypeDef *uart_handle = port->hw;


	if(port->half_duplex){
		uart_handle->Instance->CR1 &= ~USART_CR1_RE;
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(uart_handle, buf, len);
	//vTaskDelay(1);
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
#ifdef MESC_UART_USB
	xSemaphoreTake(port->tx_semaphore, portMAX_DELAY);
	while(CDC_Transmit_FS((uint8_t*)buf, len)== USBD_BUSY){
		vTaskDelay(1);
	}
	xSemaphoreGive(port->tx_semaphore);
#endif
}


void putbuffer_stream(unsigned char *buf, unsigned int len, port_str * port){
	xSemaphoreTake(port->tx_semaphore, portMAX_DELAY);
	uint32_t write_len = len;
	while(len){
		if(len > port->rx_buffer_size){
			write_len = port->rx_buffer_size;
		}else{
			write_len = len;
		}
		len -= xStreamBufferSend(port->tx_stream, buf, write_len, portMAX_DELAY);
		buf += write_len;
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
	case HW_TYPE_CAN:
		putbuffer_stream(buf, len, port);
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


uint32_t ext_printf(void * port, const char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len = 0;
	if(format != NULL){

		char send_buffer[128];
		len = vsnprintf(send_buffer, sizeof(send_buffer), format, arg);
		if(len > sizeof(send_buffer)){
			len = sizeof(send_buffer);
		}

		if(len > 0) {
			putbuffer((unsigned char*)send_buffer, len, port);
		}
	}else{
		char *s = va_arg(arg, char*);
		int len = va_arg(arg, int);
		putbuffer((unsigned char*)s, len, port);
	}

	va_end (arg);
	return len;
}

StreamBufferHandle_t rx_stream;

void USB_CDC_Callback(uint8_t *buffer, uint32_t len){
	xStreamBufferSendFromISR(rx_stream, buffer, len, NULL);
}



void CLI_init_can(port_str * port){
#ifdef HAL_CAN_MODULE_ENABLED
	CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(port->hw, &sFilterConfig) != HAL_OK)
	{
	/* Filter configuration Error */
	Error_Handler();
	}


	HAL_CAN_Start(port->hw); //start CAN
#endif
}


volatile TERMINAL_HANDLE * debug;

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
		case HW_TYPE_CAN:
			port->rx_stream = xStreamBufferCreate(port->rx_buffer_size, 1);
			port->tx_stream = xStreamBufferCreate(port->rx_buffer_size, 1);
#ifdef HAL_CAN_MODULE_ENABLED
			TASK_CAN_init(port, CAN_NAME);
#endif
			break;
	}

	port->tx_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(port->tx_semaphore);

	port->term_block = xSemaphoreCreateBinary();
	xSemaphoreGive(port->term_block);

	TERMINAL_HANDLE * term_cli = NULL;

	switch(port->hw_type){
		case HW_TYPE_UART:
			term_cli =  TERM_createNewHandle(ext_printf, port, pdTRUE, &TERM_defaultList, NULL, "uart");
			debug = term_cli;
			break;
		case HW_TYPE_USB:
			term_cli =  TERM_createNewHandle(ext_printf, port, pdTRUE, &TERM_defaultList, NULL, "usb");
			break;
		case HW_TYPE_CAN:
			term_cli =  TERM_createNewHandle(ext_printf, port, pdTRUE, &TERM_defaultList, NULL, "CAN");
			break;
	}

	if(term_cli != NULL){
		null_handle.varHandle = TERM_VAR_init(term_cli, (uint8_t*)RTOS_flash_base_address(), RTOS_flash_base_size(), RTOS_flash_clear, RTOS_flash_start_write, RTOS_flash_write, RTOS_flash_end_write);
	}

	MESCinterface_init(term_cli);

	if(port->hw_type == HW_TYPE_UART){
		rd_ptr = uart_get_write_pos(port); //Clear input buffer.
	}


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
			case HW_TYPE_CAN:
				xSemaphoreTake(port->term_block, portMAX_DELAY);
				while(xStreamBufferReceive(port->rx_stream, &c, 1, 1)){
					TERM_processBuffer(&c,1,term_cli);
				}
				xSemaphoreGive(port->term_block);
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
