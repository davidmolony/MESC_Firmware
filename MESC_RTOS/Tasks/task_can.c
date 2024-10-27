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

#include "CAN_helper.h"

#include "cmsis_os.h"
#include "string.h"

#include "init.h"

#include <stdlib.h>

#ifdef HAL_CAN_MODULE_ENABLED

#define TASK_CAN_BROADCAST 0

TASK_CAN_node * node_lut[NUM_NODES];

TASK_CAN_node last_overrun;

uint8_t num_nodes_active=0;

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


void CAN1_RX0_IRQHandler(void)
{
	TASK_CAN_packet packet;

	CAN_RxHeaderTypeDef pheader;

	HAL_CAN_GetRxMessage(can1.hw, CAN_RX_FIFO0, &pheader, packet.buffer);

	packet.len = pheader.DLC;
	packet.message_id = CANhelper_unpackMESC_id(pheader.ExtId, &packet.sender, &packet.receiver);

	if(packet.receiver == 0 || packet.receiver == can1.node_id){

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		if(xQueueSendFromISR(can1.rx_queue, &packet, &xHigherPriorityTaskWoken) != pdPASS){
			can1.rx_dropped++;
		}

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

__weak void * TASK_CAN_allocate_node(TASK_CAN_handle * handle, TASK_CAN_node * node){
	UNUSED(handle);
	return NULL;
}

__weak void * TASK_CAN_free_node(TASK_CAN_handle * handle, TASK_CAN_node * node){
	UNUSED(handle);
	return NULL;
}

void TASK_CAN_packet_received(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len){
	switch(id){
		case CAN_ID_PING:{

			TASK_CAN_node * node = TASK_CAN_get_node_from_id(sender);

			if(node == NODE_OVERRUN){
				last_overrun.id = sender;
				memcpy(last_overrun.short_name, data,8);
				last_overrun.short_name[8] = 0;
			}

			if(node==NULL && node != NODE_OVERRUN){
				node = pvPortMalloc(sizeof(TASK_CAN_node));
				if(node){
					memset(node, 0, sizeof(TASK_CAN_node));
					node_lut[sender] = node;
					node->last_seen = 3;
					node->id = sender;
					memcpy(node->short_name, data,8);
					node->short_name[8] = 0;
					TASK_CAN_allocate_node(handle, node);

				}
			}else{
				node->last_seen = 3;
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


void TASK_CAN_rx(void * argument){
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

void TASK_CAN_ping(TASK_CAN_handle * handle){
	if(HAL_CAN_GetTxMailboxesFreeLevel(handle->hw)){

		uint8_t buffer[8];
		uint32_t TxMailbox;
		PACK_8b_char_to_buf(buffer, handle->short_name);

		CAN_TxHeaderTypeDef TxHeader;
		TxHeader.ExtId = CANhelper_packMESC_id(CAN_ID_PING, handle->node_id, TASK_CAN_BROADCAST);
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.DLC = 8;
		TxHeader.TransmitGlobalTime = DISABLE;

		HAL_CAN_AddTxMessage(handle->hw, &TxHeader, buffer, &TxMailbox);  //function to add message for transmition
	}

	uint8_t active_nodes=0;
	for(uint32_t i=0;i<NUM_NODES;i++){
		TASK_CAN_node * node = TASK_CAN_get_node_from_id(i);

		if(node!=NULL && node != NODE_OVERRUN){
			node->last_seen--;
			if(node->last_seen == 0){
				if(node->data){
					TASK_CAN_free_node(handle, node);
				}
				vPortFree(node);
				node_lut[i] = NULL;
			}else{
				active_nodes++;
			}
		}
	}
	num_nodes_active = active_nodes;
	if(num_nodes_active > 0){
		CLEAR_BIT(handle->hw->Instance->MCR, CAN_MCR_NART);  //Activate automatic retransmission if at least one other node is on the bus
	}else{
		SET_BIT(handle->hw->Instance->MCR, CAN_MCR_NART);
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
		TxHeader.ExtId = CANhelper_packMESC_id(CAN_ID_CONNECT, handle->node_id, remote);
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

TASK_CAN_node * TASK_CAN_get_node_from_id(uint8_t id){
	if (id<NUM_NODES) {
		return node_lut[id];
	}else{
		return (TASK_CAN_node *)0xFFFFFFFF;
	}

}

void TASK_CAN_tx(void * argument){

	port_str * port = argument;
	TASK_CAN_handle * handle = port->hw;

	uint32_t TxMailbox;

	uint32_t last_ping = xTaskGetTickCount();

	TASK_CAN_packet packet;

	while(1){
		if(HAL_CAN_GetTxMailboxesFreeLevel(handle->hw)){
			uint8_t buffer[8];

			uint8_t len = xStreamBufferReceive(port->tx_stream, buffer, sizeof(buffer), 0);

			if(len){
				CAN_TxHeaderTypeDef TxHeader;
				TxHeader.ExtId = CANhelper_packMESC_id(CAN_ID_TERMINAL, handle->node_id, handle->remote_node_id);
				TxHeader.DLC = len;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.IDE = CAN_ID_EXT;
				TxHeader.TransmitGlobalTime = DISABLE;

				HAL_CAN_AddTxMessage(handle->hw, &TxHeader, buffer, &TxMailbox);

			}

		}

		if(HAL_CAN_GetTxMailboxesFreeLevel(handle->hw)){
			if(xQueueReceive(handle->tx_queue, &packet, 0)){

				CAN_TxHeaderTypeDef TxHeader;
				bool error = false;

				switch(packet.type){
				case CANpacket_TYPE_MESC:
					TxHeader.ExtId = CANhelper_packMESC_id(packet.message_id , packet.sender, packet.receiver);
					TxHeader.IDE = CAN_ID_EXT;
					break;
				case CANpacket_TYPE_STD:
					TxHeader.StdId = packet.message_id;
					TxHeader.IDE = CAN_ID_STD;
					break;
				case CANpacket_TYPE_EXT:
					TxHeader.ExtId = packet.message_id;
					TxHeader.IDE = CAN_ID_EXT;
					break;
				default:
					error = true;
					break;
				}

				if(error == false){
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.DLC = packet.len;
					TxHeader.TransmitGlobalTime = DISABLE;

					HAL_CAN_AddTxMessage(handle->hw, &TxHeader, packet.buffer, &TxMailbox);
				}

			}
		}

		if(xTaskGetTickCount() >  last_ping + 1000){
			TASK_CAN_ping(handle);
			last_ping = xTaskGetTickCount();
		}


		if(xStreamBufferIsEmpty(port->tx_stream) && uxQueueMessagesWaiting(handle->tx_queue) == 0){
			vTaskDelay(10);
		}
		vTaskDelay(1);
	}
}


__weak void TASK_CAN_telemetry_fast(TASK_CAN_handle * handle){
  UNUSED(handle);
}

__weak void TASK_CAN_telemetry_slow(TASK_CAN_handle * handle){
  UNUSED(handle);
}

__weak void TASK_CAN_aux_data(TASK_CAN_handle * handle){
  UNUSED(handle);
}

void TASK_CAN_telemetry(void * argument){
	uint32_t count=0;
	port_str * port = argument;
	TASK_CAN_handle * handle = port->hw;
	while(1){

		if(count % 10 == 0){
			TASK_CAN_telemetry_fast(handle);
		}
		if(count==100){
			TASK_CAN_telemetry_slow(handle);
			count=0;
		}

		count++;
		TASK_CAN_aux_data(handle);

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}


#define CAN_STREAM_SIZE 128

void TASK_CAN_init(port_str * port, char * short_name){
	TASK_CAN_handle * handle = port->hw;
	memset(handle->short_name,0,9);
	strncpy(handle->short_name, short_name, 8);

	handle->rx_queue = xQueueCreate(128, sizeof(TASK_CAN_packet));
	handle->tx_queue = xQueueCreate(128, sizeof(TASK_CAN_packet));

	xTaskCreate(TASK_CAN_rx, "task_rx_can", 256, (void*)port, osPriorityAboveNormal, &handle->rx_task_handle);
	xTaskCreate(TASK_CAN_tx, "task_tx_can", 256, (void*)port, osPriorityAboveNormal, &handle->tx_task_handle);
	xTaskCreate(TASK_CAN_telemetry, "can_metry", 256, (void*)port, osPriorityAboveNormal, NULL);
}


uint8_t CMD_nodes(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	ttprintf("Active nodes:\r\n");
	bool found = false;
	ttprintf("ID: %u\tType: %s\tThis node\r\n", can1.node_id, can1.short_name);
	for(uint32_t i=0;i<NUM_NODES;i++){
		TASK_CAN_node * node = TASK_CAN_get_node_from_id(i);
		if(node!=NULL && node != NODE_OVERRUN){
			ttprintf("ID: %u\tType: %s\tLast seen: %u\r\n", node->id, node->short_name, node->last_seen);
			found=true;
		}
	}
	if(found==false){
		ttprintf("No active nodes found\r\n\r\n");
	}

	ttprintf("Passive nodes:\r\n");

	uint32_t cnt=2000;
	uint8_t last_id=0;
	while(cnt){
		if(last_id != last_overrun.id){
			ttprintf("ID: %u\tType: %s\r\n", last_overrun.id, last_overrun.short_name);
			last_id = last_overrun.id;
		}
		cnt--;
		vTaskDelay(1);
	}

	return TERM_CMD_EXIT_SUCCESS;

}

uint8_t CMD_can_send(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	uint32_t id = 0;
	uint8_t receiver = CAN_BROADCAST;
	float f_number[2] = {0.0f, 0.0f};
	uint32_t u_number[2] = {0, 0};
	bool b_float = false;
	bool b_uint = false;


	for(int i=0;i<argCount;i++){
		if(strcmp(args[i], "-id")==0){
			if(i+1 < argCount){
				id = strtoul(args[i+1], NULL, 0);
			}
		}
		if(strcmp(args[i], "-f")==0){
			if(i+1 < argCount){
				b_float = true;
				f_number[0] = strtof(args[i+1], NULL);
				f_number[1] = 0.0f;
			}
		}
		if(strcmp(args[i], "-f2")==0){
			if(i+2 < argCount){
				b_float = true;
				f_number[0] = strtof(args[i+1], NULL);
				f_number[1] = strtof(args[i+2], NULL);
			}
		}
		if(strcmp(args[i], "-u")==0){
			if(i+1 < argCount){
				b_uint = true;
				u_number[0] = strtoul(args[i+1], NULL, 0);
				u_number[1] = 0;
			}
		}
		if(strcmp(args[i], "-u2")==0){
			if(i+2 < argCount){
				b_uint = true;
				u_number[0] = strtoul(args[i+1], NULL, 0);
				u_number[1] = strtoul(args[i+2], NULL, 0);
			}
		}
		if(strcmp(args[i], "-r")==0){
			if(i+1 < argCount){
				receiver = strtoul(args[i+1], NULL, 0);
			}
		}
		if(strcmp(args[i], "-i")==0){
			ttprintf("Dropped frames\r\nTX: %u RX: %u\r\n", can1.stream_dropped, can1.rx_dropped);
		}
	}

	if(b_float){
		TASK_CAN_add_float(&can1, id, receiver, f_number[0], f_number[1], 100);
	}

	if(b_uint){
		TASK_CAN_add_uint32(&can1, id, receiver, u_number[0], u_number[1], 100);
	}

	return TERM_CMD_EXIT_SUCCESS;

	}


#endif
