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

#include "task_can.h"
#include "task_cli.h"

#include "can_ids.h"

#include "cmsis_os.h"
#include "string.h"

#include "init.h"

#include <stdlib.h>

#ifdef HAL_CAN_MODULE_ENABLED

#define TASK_CAN_BROADCAST 0

void TASK_CAN_init_can(TASK_CAN_handle * handle){

	CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure

	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(handle->hw, &sFilterConfig) != HAL_OK)
	{
	/* Filter configuration Error */
	Error_Handler();
	}

	HAL_CAN_Start(handle->hw); //start CAN
	HAL_CAN_ActivateNotification(handle->hw, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void buffer_uint32(uint8_t* buffer, uint32_t number) {
	*buffer = number >> 24;
	buffer++;
	*buffer = number >> 16;
	buffer++;
	*buffer = number >> 8;
	buffer++;
	*buffer = number;
}

float buffer_to_float(uint8_t* buffer) {
	float ret;
	memcpy(&ret,buffer, sizeof(float));
	return ret;
}

void buffer_8b_char(uint8_t* buffer, char * short_name) {
	memcpy(buffer, short_name, 8);
}

uint32_t generate_id(uint16_t id, uint8_t sender, uint8_t receiver){
	uint32_t ret = (uint32_t)id << 16;
	ret |= sender;
	ret |= (uint32_t)receiver << 8;
	return ret;
}

uint16_t extract_id(uint32_t ext_id, uint8_t * sender, uint8_t * receiver){
	*sender = ext_id & 0xFF;
	*receiver = (ext_id >> 8) & 0xFF;
	return (ext_id >> 16);
}



TASK_CAN_nodes nodes[NUM_NODES];

typedef struct {
	uint16_t message_id;
	uint8_t sender;
	uint8_t receiver;
	uint8_t len;
	uint8_t buffer[8];
}TASK_CAN_packet;


bool TASK_CAN_add_float(TASK_CAN_handle * handle, uint16_t message_id, uint8_t receiver, float number, uint32_t timeout){
	TASK_CAN_packet packet;

	packet.message_id = message_id;
	packet.receiver = receiver;
	packet.sender = handle->node_id;
	packet.len = sizeof(float);
	memcpy(&packet.buffer, &number, sizeof(float));
	return xQueueSend(handle->tx_queue, &packet, pdMS_TO_TICKS(timeout));
}

bool TASK_CAN_add_uint32(TASK_CAN_handle * handle, uint16_t message_id, uint8_t receiver, uint32_t number, uint32_t timeout){
	TASK_CAN_packet packet;

	packet.message_id = message_id;
	packet.receiver = receiver;
	packet.sender = handle->node_id;
	packet.len = sizeof(uint32_t);
	buffer_uint32(packet.buffer, number);
	return xQueueSend(handle->tx_queue, &packet, pdMS_TO_TICKS(timeout));
}



void CAN1_RX0_IRQHandler(void)
{
	TASK_CAN_packet packet;

	CAN_RxHeaderTypeDef pheader;

	HAL_CAN_GetRxMessage(can1.hw, CAN_RX_FIFO0, &pheader, packet.buffer);

	packet.len = pheader.DLC;
	packet.message_id = extract_id(pheader.ExtId, &packet.sender, &packet.receiver);

	if(packet.receiver == 0 || packet.receiver == can1.node_id){

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(can1.rx_queue, &packet, &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

}

__weak void TASK_CAN_packet_cb(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len){
  UNUSED(handle);
  UNUSED(id);
  UNUSED(sender);
  UNUSED(receiver);
  UNUSED(data);
  UNUSED(len);
}

void TASK_CAN_packet_received(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len){
	switch(id){
		case CAN_ID_PING:{
			bool found=false;
			int32_t first_empty=-1;

			for(uint32_t i=0;i<NUM_NODES;i++){
				if(nodes[i].id == sender){
					nodes[i].last_seen = 3;
					found=true;
					break;
				}
				if(nodes[i].id == 0 && first_empty == -1) first_empty = i;
			}

			if(found==false){
				nodes[first_empty].last_seen = 3;
				nodes[first_empty].id = sender;
				memcpy(nodes[first_empty].short_name, data,8);
				nodes[first_empty].short_name[8] = 0;
			}

			break;
		}
		case CAN_ID_CONNECT:{
			if(*data){
				handle->remote_node_id = sender;
			}else{
				handle->remote_node_id = 0;
			}
			break;
		}
		default:
			TASK_CAN_packet_cb(handle, id, sender, receiver, data, len);
			break;
	}

}

#define ALLOWED_BLOCK_TIME 10





void task_rx_can(void * argument){
	port_str * port = argument;
	TASK_CAN_handle * handle = port->hw;

	TASK_CAN_init_can(handle);

	TASK_CAN_packet packet;


	while(1){
		xQueueReceive(handle->rx_queue, &packet, portMAX_DELAY);
#ifdef LED_RED_Pin
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
#endif
		if(packet.message_id == CAN_ID_TERMINAL && packet.receiver == handle->node_id){
			handle->remote_node_id = packet.sender;
			if(xStreamBufferSend(port->rx_stream, packet.buffer, packet.len, ALLOWED_BLOCK_TIME) != packet.len){
				handle->stream_dropped++;  //Streambuffer was not consumed fast enough from other tasks
			}
		}else{
			TASK_CAN_packet_received(handle, packet.message_id, packet.sender, packet.receiver, packet.buffer, packet.len);
		}
	}

}
volatile uint32_t level=0;

void TASK_CAN_ping(TASK_CAN_handle * handle){
	if(HAL_CAN_GetTxMailboxesFreeLevel(handle->hw)){

		uint8_t buffer[8];
		uint32_t TxMailbox;
		buffer_8b_char(buffer, handle->short_name);

		CAN_TxHeaderTypeDef TxHeader;
		TxHeader.ExtId = generate_id(CAN_ID_PING, handle->node_id, TASK_CAN_BROADCAST);
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.DLC = 8;
		TxHeader.TransmitGlobalTime = DISABLE;

		HAL_CAN_AddTxMessage(handle->hw, &TxHeader, buffer, &TxMailbox);  //function to add message for transmition
	}

	for(uint32_t i=0;i<NUM_NODES;i++){
		if(nodes[i].id && nodes[i].last_seen){
			nodes[i].last_seen--;
			if(nodes[i].last_seen == 0){
				nodes[i].id = 0;
				memset(nodes[i].short_name,0,9);
			}
		}
	}
}

uint32_t TASK_CAN_connect(TASK_CAN_handle * handle, uint16_t remote, uint8_t connect){
	if(HAL_CAN_GetTxMailboxesFreeLevel(handle->hw)){
		if(connect){
			handle->remote_node_id = remote;
		}else{
			handle->remote_node_id = 0;
		}

		uint8_t buffer[1];
		uint32_t TxMailbox;
		buffer[0] = connect;

		CAN_TxHeaderTypeDef TxHeader;
		TxHeader.ExtId = generate_id(CAN_ID_CONNECT, handle->node_id, remote);
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.DLC = 1;
		TxHeader.TransmitGlobalTime = DISABLE;

		HAL_CAN_AddTxMessage(handle->hw, &TxHeader, buffer, &TxMailbox);  //function to add message for transmition
		return 1;
	}else{
		return 0;
	}

}



void task_tx_can(void * argument){

	port_str * port = argument;
	TASK_CAN_handle * handle = port->hw;

	uint32_t TxMailbox;

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = 0;
	TxHeader.TransmitGlobalTime = DISABLE;


	uint32_t last_ping = xTaskGetTickCount();

	TASK_CAN_packet packet;



	while(1){
		if(HAL_CAN_GetTxMailboxesFreeLevel(handle->hw)>2){

			uint8_t buffer[8];

			uint8_t len = xStreamBufferReceive(port->tx_stream, buffer, sizeof(buffer), 0);

			if(len){

				TxHeader.ExtId = generate_id(CAN_ID_TERMINAL, handle->node_id, handle->remote_node_id);
				TxHeader.DLC = len;

				HAL_CAN_AddTxMessage(handle->hw, &TxHeader, buffer, &TxMailbox);  //function to add message for transmition

			}

		}

		if(HAL_CAN_GetTxMailboxesFreeLevel(handle->hw)){
			if(xQueueReceive(handle->tx_queue, &packet, 0)){

				CAN_TxHeaderTypeDef TxHeader;
				TxHeader.ExtId = generate_id(packet.message_id , packet.sender, packet.receiver);
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.IDE = CAN_ID_EXT;
				TxHeader.DLC = packet.len;
				TxHeader.TransmitGlobalTime = DISABLE;

				HAL_CAN_AddTxMessage(handle->hw, &TxHeader, packet.buffer, &TxMailbox);  //function to add message for transmition

			}
		}

		if(xTaskGetTickCount() >  last_ping + 1000){
			TASK_CAN_ping(handle);
			last_ping = xTaskGetTickCount();
		}

		if(xStreamBufferIsEmpty(port->tx_stream) && uxQueueMessagesWaiting(handle->tx_queue) == 0){
			vTaskDelay(10);
		}else{
			//getvTaskDelay(2);
		}

	}

}

#define CAN_STREAM_SIZE 128

void TASK_CAN_init(port_str * port, char * short_name){
	TASK_CAN_handle * handle = port->hw;
	memset(handle->short_name,0,9);
	strncpy(handle->short_name, short_name, 8);

	handle->rx_queue = xQueueCreate(128, sizeof(TASK_CAN_packet));
	handle->tx_queue = xQueueCreate(128, sizeof(TASK_CAN_packet));

	xTaskCreate(task_rx_can, "task_rx_can", 256, (void*)port, osPriorityAboveNormal, &handle->rx_task_handle);
	xTaskCreate(task_tx_can, "task_tx_can", 256, (void*)port, osPriorityAboveNormal, &handle->tx_task_handle);
}


uint8_t CMD_nodes(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	ttprintf("Active nodes:\r\n");
	bool found = false;
	ttprintf("ID: %u\tType: %s\tThis node\r\n", can1.node_id, can1.short_name);
	for(uint32_t i=0;i<NUM_NODES;i++){
		if(nodes[i].id){
			ttprintf("ID: %u\tType: %s\tLast seen: %u\r\n", nodes[i].id, nodes[i].short_name, nodes[i].last_seen);
			found=true;
		}
	}
	if(found==false){
		ttprintf("No node found\r\n");
	}
	return TERM_CMD_EXIT_SUCCESS;

}

uint8_t CMD_can_send(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	uint32_t id;
	uint8_t receiver = CAN_BROADCAST;
	float f_number;
	bool b_float = false;


	for(int i=0;i<argCount;i++){
		if(strcmp(args[i], "-id")==0){
			if(i+1 < argCount){
				id = strtoul(args[i+1], NULL, 0);
			}
		}
		if(strcmp(args[i], "-f")==0){
			if(i+1 < argCount){
				b_float = true;
				f_number = strtof(args[i+1], NULL);
			}
		}
		if(strcmp(args[i], "-r")==0){
			if(i+1 < argCount){
				receiver = strtoul(args[i+1], NULL, 0);
			}
		}
	}

	if(b_float){
		TASK_CAN_add_float(&can1, id, receiver, f_number, 100);
	}

	return TERM_CMD_EXIT_SUCCESS;

	}


#endif
