/*
 **
 ******************************************************************************
 * @file           : CAN_helper.h
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

#ifndef CAN_HELPER_H_
#define CAN_HELPER_H_

#include "CAN_types.h"
#include "can_ids.h"

#ifdef HAL_CAN_MODULE_ENABLED

bool TASK_CAN_add_float(TASK_CAN_handle * handle, uint16_t message_id, uint8_t receiver, float n1, float n2, uint32_t timeout);
bool TASK_CAN_add_uint32(TASK_CAN_handle * handle, uint16_t message_id, uint8_t receiver, uint32_t n1, uint32_t n2, uint32_t timeout);
bool TASK_CAN_add_sample(TASK_CAN_handle * handle, uint16_t message_id, uint8_t receiver, uint16_t row, uint8_t col, uint8_t flags, float value, uint32_t timeout);
bool TASK_CAN_add_rawSTD(TASK_CAN_handle * handle, uint32_t message_id, uint8_t * data, uint8_t len, uint32_t timeout);
bool TASK_CAN_add_rawEXT(TASK_CAN_handle * handle, uint32_t message_id, uint8_t * data, uint8_t len, uint32_t timeout);

#endif

float PACK_buf_to_float(uint8_t* buffer);
uint32_t PACK_buf_to_u32(uint8_t* buffer);
uint16_t PACK_buf_to_u16(uint8_t* buffer);
uint8_t PACK_buf_to_u8(uint8_t* buffer);

void PACK_u32_to_buf(uint8_t* buffer, uint32_t number);
void PACK_u16_to_buf(uint8_t* buffer, uint16_t number);
void PACK_u8_to_buf(uint8_t* buffer, uint8_t number);
void PACK_float_to_buf(uint8_t* buffer, float number);
void PACK_8b_char_to_buf(uint8_t* buffer, char * short_name);


uint32_t CANhelper_packMESC_id(uint16_t id, uint8_t sender, uint8_t receiver);
uint16_t CANhelper_unpackMESC_id(uint32_t ext_id, uint8_t * sender, uint8_t * receiver);
uint32_t CANhelper_packJ1939_id(uint8_t priority, uint8_t data_page, uint8_t PDU_format, uint8_t PDU_specific, uint8_t source_address);

#endif /* CAN_HELPER_H_ */
