/*
 **
 ******************************************************************************
 * @file           : CAN_helper.c
 * @brief          : Helper functions for CAN
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

#include "CAN_helper.h"
#include "string.h"

#ifdef HAL_CAN_MODULE_ENABLED
bool TASK_CAN_add_float(TASK_CAN_handle * handle, uint16_t message_id, uint8_t receiver, float n1, float n2, uint32_t timeout){
	TASK_CAN_packet packet;

	packet.type = CANpacket_TYPE_MESC;
	packet.message_id = message_id;
	packet.receiver = receiver;
	packet.sender = handle->node_id;
	packet.len = sizeof(float)*2;
	memcpy(&packet.buffer[0], &n1, sizeof(float));
	memcpy(&packet.buffer[4], &n2, sizeof(float));
	return xQueueSend(handle->tx_queue, &packet, pdMS_TO_TICKS(timeout));
}

bool TASK_CAN_add_uint32(TASK_CAN_handle * handle, uint16_t message_id, uint8_t receiver, uint32_t n1, uint32_t n2, uint32_t timeout){
	TASK_CAN_packet packet;

	packet.type = CANpacket_TYPE_MESC;
	packet.message_id = message_id;
	packet.receiver = receiver;
	packet.sender = handle->node_id;
	packet.len = sizeof(uint32_t)*2;
	PACK_u32_to_buf(&packet.buffer[0], n1);
	PACK_u32_to_buf(&packet.buffer[4], n2);
	return xQueueSend(handle->tx_queue, &packet, pdMS_TO_TICKS(timeout));
}

bool TASK_CAN_add_rawSTD(TASK_CAN_handle * handle, uint32_t message_id, uint8_t * data, uint8_t len, uint32_t timeout){
	if(len>8) return false;
	if(data == NULL) return false;
	if(message_id > 0x7FF) return false;

	TASK_CAN_packet packet;

	packet.type = CANpacket_TYPE_STD;
	packet.message_id = message_id;
	packet.len = len;
	memcpy(packet.buffer, data, len);
	return xQueueSend(handle->tx_queue, &packet, pdMS_TO_TICKS(timeout));
}

bool TASK_CAN_add_rawEXT(TASK_CAN_handle * handle, uint32_t message_id, uint8_t * data, uint8_t len, uint32_t timeout){
	if(len>8) return false;
	if(data == NULL) return false;
	if(message_id > 0x1FFFFFFF) return false;

	TASK_CAN_packet packet;

	packet.type = CANpacket_TYPE_EXT;
	packet.message_id = message_id;
	packet.len = len;
	memcpy(packet.buffer, data, len);
	return xQueueSend(handle->tx_queue, &packet, pdMS_TO_TICKS(timeout));
}

bool TASK_CAN_add_sample(TASK_CAN_handle * handle, uint16_t message_id, uint8_t receiver, uint16_t row, uint8_t col, uint8_t flags, float value, uint32_t timeout){
	TASK_CAN_packet packet;

	packet.type = CANpacket_TYPE_MESC;
	packet.message_id = message_id;
	packet.receiver = receiver;
	packet.sender = handle->node_id;
	packet.len = 8; //Full 8 byte
	PACK_u16_to_buf(&packet.buffer[0], row);
	PACK_u8_to_buf(&packet.buffer[2], col);
	PACK_u8_to_buf(&packet.buffer[3], flags);
	PACK_float_to_buf(&packet.buffer[4], value);
	return xQueueSend(handle->tx_queue, &packet, pdMS_TO_TICKS(timeout));
}
#endif

void PACK_u32_to_buf(uint8_t* buffer, uint32_t number) {
	*buffer = number >> 24;
	buffer++;
	*buffer = number >> 16;
	buffer++;
	*buffer = number >> 8;
	buffer++;
	*buffer = number;
}

void PACK_u16_to_buf(uint8_t* buffer, uint16_t number) {
	*buffer = number >> 8;
	buffer++;
	*buffer = number;
}

void PACK_u8_to_buf(uint8_t* buffer, uint8_t number) {
	*buffer = number;
}

void PACK_float_to_buf(uint8_t* buffer, float number) {
	memcpy(buffer, &number, sizeof(float));
}

uint32_t PACK_buf_to_u32(uint8_t* buffer) {
	uint32_t number;
	number = (uint32_t)*buffer << 24;
	buffer++;
	number |= (uint32_t)*buffer << 16;
	buffer++;
	number |= (uint32_t)*buffer << 8;
	buffer++;
	number |= (uint32_t)*buffer;
	return number;
}

uint16_t PACK_buf_to_u16(uint8_t* buffer) {
	uint16_t number;
	number = (uint16_t)*buffer << 8;
	buffer++;
	number |= (uint16_t)*buffer;
	return number;
}

uint8_t PACK_buf_to_u8(uint8_t* buffer) {
	uint8_t number;
	number = *buffer;
	return number;
}

float PACK_buf_to_float(uint8_t* buffer) {
	float ret;
	memcpy(&ret,buffer, sizeof(float));
	return ret;
}

void PACK_8b_char_to_buf(uint8_t* buffer, char * short_name) {
	memcpy(buffer, short_name, 8);
}

uint32_t CANhelper_packMESC_id(uint16_t id, uint8_t sender, uint8_t receiver){
	uint32_t ret = (uint32_t)id << 16;
	ret |= sender;
	ret |= (uint32_t)receiver << 8;
	return ret;
}

uint16_t CANhelper_unpackMESC_id(uint32_t ext_id, uint8_t * sender, uint8_t * receiver){
	*sender = ext_id & 0xFF;
	*receiver = (ext_id >> 8) & 0xFF;
	return (ext_id >> 16);
}

uint32_t CANhelper_packJ1939_id(uint8_t priority, uint8_t data_page, uint8_t PDU_format, uint8_t PDU_specific, uint8_t source_address){
    uint32_t ret;

    ret = ((priority & 0b111) << 26) | (data_page & 0b11) <<24 | PDU_format << 16 | PDU_specific << 8 | source_address;

    return ret;
}

