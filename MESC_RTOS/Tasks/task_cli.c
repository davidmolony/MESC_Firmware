/*
 * m365
 *
 * Copyright (c) 2021 Jens Kerrinnes
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

	term_cli =  TERM_createNewHandle(ext_printf, port, pdTRUE, &TERM_defaultList, NULL, "usb");

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
		xTaskCreate(task_cli, "tskCLI", 2048, (void*)port, osPriorityNormal, &port->task_handle);
	}
}

void task_cli_kill(port_str * port){
	if(port->task_handle){
		xTaskNotify(port->task_handle, 0, eIncrement);
		vTaskDelay(200);
	}
}
