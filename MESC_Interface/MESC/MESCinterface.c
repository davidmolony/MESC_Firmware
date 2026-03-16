
/*
 **
 ******************************************************************************
 * @file           : MESCinterface.c
 * @brief          : Initializing RTOS system and parameters
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
#include "TTerm/Core/include/TTerm.h"
#include "Common/RTOS_flash.h"
#include "MESCmotor_state.h"
#include "MESCmotor.h"
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>
#include <ctype.h>
#include <MESC/MESCinterface.h>
#include "mesc_persist.h"
#include "Tasks/task_can.h"
#include "Tasks/CAN_helper.h"
#include "usbd_cdc_if.h"
#include "TTerm/Core/include/TTerm_var.h"

#include "MESCmeasure.h"
#include "MESCinput.h"

static uint32_t null_printf(void * port, const char* format, ...) {
	va_list arg;
	UNUSED(port);
	UNUSED(format);
	va_start(arg, format);
	va_end(arg);
	return 0;
}

TERMINAL_HANDLE null_handle = {
		.print = null_printf,
		.currPermissionLevel = 0
};

/* TTerm_var expects these symbols from TTerm.c; keep minimal stubs in
   bare-metal builds where the full terminal task is removed. */
TermVariableDescriptor TERM_varList = {
		.nextVar = 0,
		.nameLength = 0
};

void TERM_sendVT100Code(TERMINAL_HANDLE * handle, uint16_t cmd, uint8_t var){
	UNUSED(handle);
	UNUSED(cmd);
	UNUSED(var);
}

#define USB_CLI_LINE_MAX 96U
#define USB_CLI_HISTORY_DEPTH 8U
#define USB_CLI_RESP_MAX APP_TX_DATA_SIZE

static char s_usb_cli_line[USB_CLI_LINE_MAX];
static char s_usb_cli_resp[USB_CLI_RESP_MAX];
static char s_usb_cli_history[USB_CLI_HISTORY_DEPTH][USB_CLI_LINE_MAX];
static char s_usb_cli_history_draft[USB_CLI_LINE_MAX];
static uint8_t s_usb_cli_tx[APP_TX_DATA_SIZE];
static uint32_t s_usb_cli_line_len = 0U;
static uint32_t s_usb_cli_cursor = 0U;
static uint8_t s_usb_cli_esc_state = 0U;
static uint8_t s_usb_cli_skip_lf = 0U;
static uint32_t s_usb_cli_history_count = 0U;
static uint32_t s_usb_cli_history_next = 0U;
static uint32_t s_usb_cli_history_draft_len = 0U;
static int32_t s_usb_cli_history_pos = -1;
static bool mesc_persist_loaded;

#ifdef HAL_CAN_MODULE_ENABLED
extern TASK_CAN_handle can1;
extern CAN_HandleTypeDef hcan1;
void TASK_CAN_packet_cb(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len);
#endif

static TermVariableDescriptor * mesc_find_var_by_name(TermVariableDescriptor *head, char const *name);
static bool mesc_cli_parse_float(char const *value, float *out);

#ifdef HAL_CAN_MODULE_ENABLED
static bool s_can_hw_ready = false;
static uint32_t s_can_iqreq_rx_count = 0U;

static uint32_t mesc_pack_can_id(uint16_t id, uint8_t sender, uint8_t receiver){
	uint32_t ret = (uint32_t)id << 16;
	ret |= sender;
	ret |= (uint32_t)receiver << 8;
	return ret;
}

static uint16_t mesc_unpack_can_id(uint32_t ext_id, uint8_t *sender, uint8_t *receiver){
	if(sender != NULL){
		*sender = (uint8_t)(ext_id & 0xFFU);
	}
	if(receiver != NULL){
		*receiver = (uint8_t)((ext_id >> 8) & 0xFFU);
	}
	return (uint16_t)(ext_id >> 16);
}

static float mesc_unpack_float(uint8_t const *buffer){
	float ret = 0.0f;
	if(buffer != NULL){
		memcpy(&ret, buffer, sizeof(float));
	}
	return ret;
}

static void mesc_can_hw_init_if_needed(void){
	CAN_FilterTypeDef sFilterConfig;

	if(s_can_hw_ready){
		return;
	}

	can1.hw = &hcan1;
	if(can1.node_id == 0U || can1.node_id > 254U){
		can1.node_id = 1U;
	}

	memset(&sFilterConfig, 0, sizeof(sFilterConfig));
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(can1.hw, &sFilterConfig) != HAL_OK){
		return;
	}

	if(HAL_CAN_Start(can1.hw) != HAL_OK){
		return;
	}

	s_can_hw_ready = true;
}

static bool mesc_can_send_posvel_frame(TASK_CAN_handle *handle, float pos_rad, float vel_rad){
	CAN_TxHeaderTypeDef tx_header;
	uint8_t buffer[8];
	uint32_t tx_mailbox;

	if(handle == NULL || handle->hw == NULL){
		return false;
	}

	if(HAL_CAN_GetTxMailboxesFreeLevel(handle->hw) == 0U){
		handle->tx_mailbox_full++;
		return false;
	}

	memcpy(&buffer[0], &pos_rad, sizeof(float));
	memcpy(&buffer[4], &vel_rad, sizeof(float));

	tx_header.ExtId = mesc_pack_can_id(CAN_ID_POSVEL, (uint8_t)handle->node_id, CAN_BROADCAST);
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.IDE = CAN_ID_EXT;
	tx_header.DLC = 8;
	tx_header.TransmitGlobalTime = DISABLE;

	if(HAL_CAN_AddTxMessage(handle->hw, &tx_header, buffer, &tx_mailbox) == HAL_OK){
		handle->tx_frames_sent++;
		return true;
	}

	handle->tx_frames_failed++;
	return false;
}

static bool mesc_can_send_two_float_frame(TASK_CAN_handle *handle, uint16_t message_id, float v0, float v1){
	CAN_TxHeaderTypeDef tx_header;
	uint8_t buffer[8];
	uint32_t tx_mailbox;

	if(handle == NULL || handle->hw == NULL){
		return false;
	}

	if(HAL_CAN_GetTxMailboxesFreeLevel(handle->hw) == 0U){
		handle->tx_mailbox_full++;
		return false;
	}

	memcpy(&buffer[0], &v0, sizeof(float));
	memcpy(&buffer[4], &v1, sizeof(float));

	tx_header.ExtId = mesc_pack_can_id(message_id, (uint8_t)handle->node_id, CAN_BROADCAST);
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.IDE = CAN_ID_EXT;
	tx_header.DLC = 8;
	tx_header.TransmitGlobalTime = DISABLE;

	if(HAL_CAN_AddTxMessage(handle->hw, &tx_header, buffer, &tx_mailbox) == HAL_OK){
		handle->tx_frames_sent++;
		return true;
	}

	handle->tx_frames_failed++;
	return false;
}

/*
 * Some builds do not run the legacy TASK_CAN_rx queue/task path, so IQREQ
 * frames can be present in FIFO0 without reaching TASK_CAN_packet_cb.
 * Polling here keeps RX handling in the known-active interface periodic loop
 * while preserving the existing packet callback logic.
 */
static void mesc_can_poll_rx_fifo(TASK_CAN_handle *handle){
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	uint8_t sender;
	uint8_t receiver;
	uint16_t id;
	uint32_t frames_polled = 0U;
	uint32_t fill_level;
	const uint32_t max_frames_per_call = 8U;

	if(handle == NULL || handle->hw == NULL){
		return;
	}

	while(frames_polled < max_frames_per_call){
		fill_level = HAL_CAN_GetRxFifoFillLevel(handle->hw, CAN_RX_FIFO0);
		if(fill_level == 0U){
			break;
		}

		if(HAL_CAN_GetRxMessage(handle->hw, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK){
			handle->rx_isr_getmsg_err++;
			break;
		}

		handle->rx_packets_handled++;
		sender = 0U;
		receiver = 0U;
		id = mesc_unpack_can_id(rx_header.ExtId, &sender, &receiver);

		if(receiver != CAN_BROADCAST && receiver != handle->node_id){
			handle->rx_isr_filtered++;
			frames_polled++;
			continue;
		}

		if(rx_header.DLC > sizeof(rx_data)){
			handle->rx_dropped++;
			frames_polled++;
			continue;
		}

		TASK_CAN_packet_cb(handle, id, sender, receiver, rx_data, rx_header.DLC);
		frames_polled++;
	}

	fill_level = HAL_CAN_GetRxFifoFillLevel(handle->hw, CAN_RX_FIFO0);
	if(fill_level > 0U){
		handle->rx_dropped += fill_level;
	}
}

// Send key motor status CAN frames at 100Hz
void TASK_CAN_telemetry_motor_status(TASK_CAN_handle *handle) {
	MESC_motor_typedef *motor_curr;

	if(handle == NULL){
		return;
	}

	motor_curr = &mtr[0];
	(void)mesc_can_send_two_float_frame(handle, CAN_ID_SPEED, motor_curr->FOC.eHz, 0.0f);
	(void)mesc_can_send_two_float_frame(handle, CAN_ID_BUS_VOLT_CURR, motor_curr->Conv.Vbus, motor_curr->FOC.Ibus);
	(void)mesc_can_send_two_float_frame(handle, CAN_ID_MOTOR_CURRENT, motor_curr->FOC.Idq.q, motor_curr->FOC.Idq.d);
	(void)mesc_can_send_two_float_frame(handle, CAN_ID_MOTOR_VOLTAGE, motor_curr->FOC.Vdq.q, motor_curr->FOC.Vdq.d);
	(void)mesc_can_send_two_float_frame(handle, CAN_ID_TEMP_MOT_MOS1, motor_curr->Conv.Motor_T, motor_curr->Conv.MOSu_T);
}
#endif

static char const * mesc_cli_motor_state_name(motor_state_e state){
	switch(state){
	case MOTOR_STATE_INITIALISING: return "INITIALISING";
	case MOTOR_STATE_DETECTING: return "DETECTING";
	case MOTOR_STATE_ALIGN: return "ALIGN";
	case MOTOR_STATE_MEASURING: return "MEASURING";
	case MOTOR_STATE_OPEN_LOOP_STARTUP: return "OPEN_LOOP_STARTUP";
	case MOTOR_STATE_OPEN_LOOP_TRANSITION: return "OPEN_LOOP_TRANSITION";
	case MOTOR_STATE_TRACKING: return "TRACKING";
	case MOTOR_STATE_RUN: return "RUN";
	case MOTOR_STATE_GET_KV: return "GET_KV";
	case MOTOR_STATE_TEST: return "TEST";
	case MOTOR_STATE_ERROR: return "ERROR";
	case MOTOR_STATE_RECOVERING: return "RECOVERING";
	case MOTOR_STATE_SLAMBRAKE: return "SLAMBRAKE";
	case MOTOR_STATE_IDLE: return "IDLE";
	case MOTOR_STATE_RUN_BLDC: return "RUN_BLDC";
	default: return "UNKNOWN";
	}
}

static bool mesc_cli_write_allowed(void){
	motor_state_e state = mtr[0].MotorState;

	if((state == MOTOR_STATE_INITIALISING) ||
	   (state == MOTOR_STATE_TRACKING) ||
	   (state == MOTOR_STATE_IDLE) ||
	   (state == MOTOR_STATE_RUN) ||
	   (state == MOTOR_STATE_RUN_BLDC) ||
	   (state == MOTOR_STATE_ERROR)){
		return true;
	}

	return false;
}

static bool mesc_cli_uart_req_safe_override(char *name, char *value){
	float req;

	if(name == NULL || value == NULL){
		return false;
	}

	if(strcmp(name, "uart_req") != 0){
		return false;
	}

	if(!mesc_cli_parse_float(value, &req)){
		return false;
	}

	if(req < -40.0f || req > 40.0f){
		return false;
	}

	return true;
}

static bool mesc_cli_parse_uint(char const *value, uint32_t *out){
	char *end = NULL;
	unsigned long parsed;

	if(value == NULL || out == NULL || *value == '\0'){
		return false;
	}

	parsed = strtoul(value, &end, 0);
	if(end == value){
		return false;
	}

	while(end != NULL && *end != '\0'){
		if(!isspace((int)(unsigned char)*end)){
			return false;
		}
		end++;
	}

	*out = (uint32_t)parsed;
	return true;
}

static bool mesc_cli_parse_int(char const *value, int32_t *out){
	char *end = NULL;
	long parsed;

	if(value == NULL || out == NULL || *value == '\0'){
		return false;
	}

	parsed = strtol(value, &end, 0);
	if(end == value){
		return false;
	}

	while(end != NULL && *end != '\0'){
		if(!isspace((int)(unsigned char)*end)){
			return false;
		}
		end++;
	}

	*out = (int32_t)parsed;
	return true;
}

static bool mesc_cli_parse_float(char const *value, float *out){
	char *end = NULL;
	float parsed;
	float mul = 1.0f;

	if(value == NULL || out == NULL || *value == '\0'){
		return false;
	}

	parsed = strtof(value, &end);
	if(end == value){
		return false;
	}

	if(end != NULL && *end != '\0' && !isspace((int)(unsigned char)*end)){
		switch(*end){
		case 'u':
			mul = 1e-6f;
			break;
		case 'm':
			mul = 1e-3f;
			break;
		case 'k':
			mul = 1e3f;
			break;
		case 'M':
			mul = 1e6f;
			break;
		default:
			return false;
		}
		end++;
	}

	while(end != NULL && *end != '\0'){
		if(!isspace((int)(unsigned char)*end)){
			return false;
		}
		end++;
	}

	*out = parsed * mul;
	return true;
}

static bool mesc_cli_set_value(TermVariableDescriptor *var, char *value, char *out, uint32_t out_len){
	uint32_t u;
	int32_t s;
	float f;

	if(var == NULL || value == NULL || out == NULL || out_len == 0U){
		return false;
	}

	switch(var->type){
	case TERM_VARIABLE_UINT:
		if(!mesc_cli_parse_uint(value, &u)){
			snprintf(out, out_len, "ERR PARSE\r\n");
			return true;
		}
		if(u < var->min_unsigned || u > var->max_unsigned){
			snprintf(out, out_len, "ERR RANGE\r\n");
			return true;
		}
		switch(var->typeSize){
		case 1:
			*(uint8_t*)var->variable = (uint8_t)u;
			break;
		case 2:
			*(uint16_t*)var->variable = (uint16_t)u;
			break;
		case 4:
			*(uint32_t*)var->variable = u;
			break;
		default:
			snprintf(out, out_len, "ERR TYPE\r\n");
			return true;
		}
		break;
	case TERM_VARIABLE_INT:
		if(!mesc_cli_parse_int(value, &s)){
			snprintf(out, out_len, "ERR PARSE\r\n");
			return true;
		}
		if(s < var->min_signed || s > var->max_signed){
			snprintf(out, out_len, "ERR RANGE\r\n");
			return true;
		}
		switch(var->typeSize){
		case 1:
			*(int8_t*)var->variable = (int8_t)s;
			break;
		case 2:
			*(int16_t*)var->variable = (int16_t)s;
			break;
		case 4:
			*(int32_t*)var->variable = s;
			break;
		default:
			snprintf(out, out_len, "ERR TYPE\r\n");
			return true;
		}
		break;
	case TERM_VARIABLE_FLOAT:
		if(!mesc_cli_parse_float(value, &f)){
			snprintf(out, out_len, "ERR PARSE\r\n");
			return true;
		}
		if(f < var->min_float || f > var->max_float){
			snprintf(out, out_len, "ERR RANGE\r\n");
			return true;
		}
		*(float*)var->variable = f;
		break;
	case TERM_VARIABLE_CHAR:
		if(value[0] == '\0'){
			snprintf(out, out_len, "ERR PARSE\r\n");
			return true;
		}
		*(char*)var->variable = value[0];
		break;
	case TERM_VARIABLE_STRING:
		if(var->typeSize == 0U){
			snprintf(out, out_len, "ERR TYPE\r\n");
			return true;
		}
		strncpy((char*)var->variable, value, var->typeSize);
		((char*)var->variable)[var->typeSize - 1U] = '\0';
		break;
	case TERM_VARIABLE_BOOL:
		if((strcasecmp(value, "true") == 0) || (strcmp(value, "1") == 0)){
			*(bool*)var->variable = true;
		}else if((strcasecmp(value, "false") == 0) || (strcmp(value, "0") == 0)){
			*(bool*)var->variable = false;
		}else{
			snprintf(out, out_len, "ERR PARSE\r\n");
			return true;
		}
		break;
	default:
		snprintf(out, out_len, "ERR TYPE\r\n");
		return true;
	}

	if(var->cb != NULL){
		var->cb(var);
	}

	snprintf(out, out_len, "OK SET\r\n");
	return true;
}

static bool mesc_cli_write_set(char *name, char *value, char *out, uint32_t out_len){
	TermVariableDescriptor *head;
	TermVariableDescriptor *var;

	if(name == NULL || name[0] == '\0' || value == NULL || value[0] == '\0'){
		snprintf(out, out_len, "ERR ARG\r\n");
		return true;
	}

	if(null_handle.varHandle == NULL || null_handle.varHandle->varListHead == NULL){
		snprintf(out, out_len, "ERR UNINIT\r\n");
		return true;
	}

	head = null_handle.varHandle->varListHead;
	var = mesc_find_var_by_name(head, name);
	if(var == NULL){
		snprintf(out, out_len, "ERR NOT_FOUND\r\n");
		return true;
	}

	if((var->rw & VAR_ACCESS_W) == 0U){
		snprintf(out, out_len, "ERR RO\r\n");
		return true;
	}

	if(!mesc_cli_write_allowed() && !mesc_cli_uart_req_safe_override(name, value)){
		snprintf(out, out_len, "ERR UNSAFE\r\n");
		return true;
	}

	return mesc_cli_set_value(var, value, out, out_len);
}

static bool mesc_cli_write_save(char *out, uint32_t out_len){
	if(!mesc_cli_write_allowed()){
		snprintf(out, out_len, "ERR UNSAFE\r\n");
		return true;
	}

	if(CMD_varSave(&null_handle, 0, NULL) == TERM_CMD_EXIT_ERROR){
		snprintf(out, out_len, "ERR SAVE\r\n");
		return true;
	}

	snprintf(out, out_len, "OK SAVE\r\n");
	return true;
}

static bool mesc_cli_write_load(char *out, uint32_t out_len){
	if(!mesc_cli_write_allowed()){
		snprintf(out, out_len, "ERR UNSAFE\r\n");
		return true;
	}

	if(CMD_varLoad(&null_handle, 0, NULL) == TERM_CMD_EXIT_ERROR){
		snprintf(out, out_len, "ERR LOAD\r\n");
		return true;
	}

	calculateGains(&mtr[0]);
	calculateVoltageGain(&mtr[0]);
	calculateFlux(&mtr[0]);
	MESCinput_Init(&mtr[0]);
	mesc_persist_loaded = true;

	snprintf(out, out_len, "OK LOAD\r\n");
	return true;
}

static bool mesc_cli_write_list(char *out, uint32_t out_len){
	TermVariableDescriptor *curr;
	uint32_t used = 0U;
	uint32_t count = 0U;

	if(null_handle.varHandle == NULL || null_handle.varHandle->varListHead == NULL){
		snprintf(out, out_len, "ERR UNINIT\r\n");
		return true;
	}

	curr = null_handle.varHandle->varListHead;
	used = (uint32_t)snprintf(out, out_len, "OK LIST");
	if(used >= out_len){
		snprintf(out, out_len, "ERR OVERFLOW\r\n");
		return true;
	}

	while(curr != NULL){
		int n;

		if(curr->name != NULL && curr->nameLength > 0U){
			n = snprintf(&out[used], out_len - used, " %s", curr->name);
			if(n < 0 || (uint32_t)n >= (out_len - used)){
				snprintf(out, out_len, "ERR OVERFLOW\r\n");
				return true;
			}
			used += (uint32_t)n;
			count++;
		}

		curr = curr->nextVar;
	}

	if((used + 4U) >= out_len){
		snprintf(out, out_len, "ERR OVERFLOW\r\n");
		return true;
	}

	/* Keep output compact and machine-friendly. */
	(void)count;
	out[used++] = '\r';
	out[used++] = '\n';
	out[used] = '\0';
	return true;
}
// OWEN XXXX
static bool mesc_cli_write_status(char *out, uint32_t out_len){
	motor_state_e state = mtr[0].MotorState;
	char const *state_name = mesc_cli_motor_state_name(state);
	uint8_t write_allowed = 0U;
	unsigned long can_rx_drop = 0UL;
	unsigned long can_iqreq_rx = 0UL;
	unsigned long can_tx_fail = 0UL;
	unsigned long can_tx_qdrop = 0UL;
	unsigned long can_posvel_sent = 0UL;
	unsigned long can_posvel_miss = 0UL;
	unsigned long can_posvel_block = 0UL;

	if((state == MOTOR_STATE_INITIALISING) ||
	   (state == MOTOR_STATE_TRACKING) ||
	   (state == MOTOR_STATE_IDLE) ||
	   (state == MOTOR_STATE_RUN) ||
	   (state == MOTOR_STATE_RUN_BLDC) ||
	   (state == MOTOR_STATE_ERROR)){
		write_allowed = 1U;
	}

#ifdef HAL_CAN_MODULE_ENABLED
	can_rx_drop = (unsigned long)can1.rx_dropped;
	can_iqreq_rx = (unsigned long)s_can_iqreq_rx_count;
	can_tx_fail = (unsigned long)can1.tx_frames_failed;
	can_tx_qdrop = (unsigned long)can1.tx_enqueue_dropped;
	can_posvel_sent = (unsigned long)can1.posvel_sent;
	can_posvel_miss = (unsigned long)can1.posvel_slot_miss;
	can_posvel_block = (unsigned long)can1.posvel_tx_blocked;
#endif

	snprintf(out, out_len,
			 "OK STATUS state=%u state_name=%s write=%u load=%u rx_drop=%lu iqreq_rx=%lu tx_fail=%lu tx_qdrop=%lu pos_sent=%lu pos_miss=%lu pos_block=%lu\r\n",
			 (unsigned)state,
			 state_name,
			 (unsigned)write_allowed,
			 (unsigned)(mesc_persist_loaded ? 1U : 0U),
			 can_rx_drop,
			 can_iqreq_rx,
			 can_tx_fail,
			 can_tx_qdrop,
			 can_posvel_sent,
			 can_posvel_miss,
			 can_posvel_block);
	return true;
}

static bool mesc_cli_write_get(char *name, char *out, uint32_t out_len){
	TermVariableDescriptor *head;
	TermVariableDescriptor *var;
	char val[80];

	if(name == NULL || name[0] == '\0'){
		return mesc_cli_write_list(out, out_len);
	}

	if(null_handle.varHandle == NULL || null_handle.varHandle->varListHead == NULL){
		snprintf(out, out_len, "ERR UNINIT\r\n");
		return true;
	}

	head = null_handle.varHandle->varListHead;
	var = mesc_find_var_by_name(head, name);
	if(var == NULL){
		snprintf(out, out_len, "ERR NOT_FOUND\r\n");
		return true;
	}

	memset(val, 0, sizeof(val));
	if(var->type == TERM_VARIABLE_FLOAT){
		snprintf(out, out_len, "OK %s=%.6f\r\n", name, (double)(*(float*)var->variable));
		return true;
	}
	(void)TERM_var2str(&null_handle, var, val, (int32_t)sizeof(val));
	snprintf(out, out_len, "OK %s=%s\r\n", name, val);
	return true;
}

static bool mesc_cli_minimal_dispatch(char *line, char *out, uint32_t out_len){
	char *start = line;
	char *end;
	char *arg = NULL;
	char *arg2 = NULL;
	char *ws;

	while(*start == ' ' || *start == '\t'){
		start++;
	}

	if(*start == '\0'){
		return false;
	}

	end = start + strlen(start);
	while(end > start && (end[-1] == ' ' || end[-1] == '\t')){
		end--;
	}
	*end = '\0';

	ws = strpbrk(start, " \t");
	if(ws != NULL){
		*ws = '\0';
		arg = ws + 1;
		while(*arg == ' ' || *arg == '\t'){
			arg++;
		}
		if(*arg == '\0'){
			arg = NULL;
		}else{
			char *ws2 = strpbrk(arg, " \t");
			if(ws2 != NULL){
				*ws2 = '\0';
				arg2 = ws2 + 1;
				while(*arg2 == ' ' || *arg2 == '\t'){
					arg2++;
				}
				if(*arg2 == '\0'){
					arg2 = NULL;
				}
			}
		}
	}

	if(strcasecmp(start, "help") == 0){
		snprintf(out, out_len, "OK HELP HELP LIST GET [name] SET SAVE LOAD STATUS\r\n");
		return true;
	}

	if(strcasecmp(start, "list") == 0){
		return mesc_cli_write_list(out, out_len);
	}

	if(strcasecmp(start, "status") == 0){
		return mesc_cli_write_status(out, out_len);
	}

	if(strcasecmp(start, "get") == 0){
		return mesc_cli_write_get(arg, out, out_len);
	}

	if(strcasecmp(start, "set") == 0){
		return mesc_cli_write_set(arg, arg2, out, out_len);
	}

	if(strcasecmp(start, "save") == 0){
		return mesc_cli_write_save(out, out_len);
	}

	if(strcasecmp(start, "load") == 0){
		return mesc_cli_write_load(out, out_len);
	}

	snprintf(out, out_len, "ERR UNKNOWN\r\n");
	return true;
}

static void mesc_cli_tx_append_byte(uint32_t *txlen, uint8_t byte){
	if(*txlen < sizeof(s_usb_cli_tx)){
		s_usb_cli_tx[*txlen] = byte;
		(*txlen)++;
	}
}

static void mesc_cli_tx_append_data(uint32_t *txlen, uint8_t const *data, uint32_t data_len){
	uint32_t i;

	for(i = 0U; i < data_len; i++){
		mesc_cli_tx_append_byte(txlen, data[i]);
	}
}

static void mesc_cli_tx_append_str(uint32_t *txlen, char const *text){
	if(text == NULL){
		return;
	}

	mesc_cli_tx_append_data(txlen, (uint8_t const *)text, (uint32_t)strlen(text));
}

static void mesc_cli_redraw_line(uint32_t *txlen){
	uint32_t backtrack;

	mesc_cli_tx_append_byte(txlen, '\r');
	mesc_cli_tx_append_str(txlen, "\x1b[2K");
	mesc_cli_tx_append_data(txlen, (uint8_t const *)s_usb_cli_line, s_usb_cli_line_len);

	backtrack = s_usb_cli_line_len - s_usb_cli_cursor;
	while(backtrack > 0U){
		mesc_cli_tx_append_byte(txlen, '\b');
		backtrack--;
	}
}

static uint32_t mesc_cli_history_slot(uint32_t chronological_index){
	uint32_t start = (s_usb_cli_history_next + USB_CLI_HISTORY_DEPTH - s_usb_cli_history_count) % USB_CLI_HISTORY_DEPTH;
	return (start + chronological_index) % USB_CLI_HISTORY_DEPTH;
}

static void mesc_cli_set_line(char const *text){
	uint32_t copy_len = 0U;

	if(text != NULL){
		copy_len = (uint32_t)strlen(text);
		if(copy_len >= USB_CLI_LINE_MAX){
			copy_len = USB_CLI_LINE_MAX - 1U;
		}
		memcpy(s_usb_cli_line, text, copy_len);
	}

	s_usb_cli_line[copy_len] = '\0';
	s_usb_cli_line_len = copy_len;
	s_usb_cli_cursor = copy_len;
}

static void mesc_cli_store_history(void){
	uint32_t newest_slot;

	if(s_usb_cli_line_len == 0U){
		return;
	}

	if(s_usb_cli_history_count > 0U){
		newest_slot = mesc_cli_history_slot(s_usb_cli_history_count - 1U);
		if(strcmp(s_usb_cli_history[newest_slot], s_usb_cli_line) == 0){
			return;
		}
	}

	memcpy(s_usb_cli_history[s_usb_cli_history_next], s_usb_cli_line, s_usb_cli_line_len);
	s_usb_cli_history[s_usb_cli_history_next][s_usb_cli_line_len] = '\0';
	s_usb_cli_history_next = (s_usb_cli_history_next + 1U) % USB_CLI_HISTORY_DEPTH;
	if(s_usb_cli_history_count < USB_CLI_HISTORY_DEPTH){
		s_usb_cli_history_count++;
	}
}

static void mesc_cli_history_up(uint32_t *txlen){
	uint32_t slot;

	if(s_usb_cli_history_count == 0U){
		return;
	}

	if(s_usb_cli_history_pos < 0){
		memcpy(s_usb_cli_history_draft, s_usb_cli_line, s_usb_cli_line_len);
		s_usb_cli_history_draft[s_usb_cli_line_len] = '\0';
		s_usb_cli_history_draft_len = s_usb_cli_line_len;
		s_usb_cli_history_pos = (int32_t)s_usb_cli_history_count - 1;
	}else if(s_usb_cli_history_pos > 0){
		s_usb_cli_history_pos--;
	}

	slot = mesc_cli_history_slot((uint32_t)s_usb_cli_history_pos);
	mesc_cli_set_line(s_usb_cli_history[slot]);
	mesc_cli_redraw_line(txlen);
}

static void mesc_cli_history_down(uint32_t *txlen){
	uint32_t slot;

	if(s_usb_cli_history_pos < 0){
		return;
	}

	if(s_usb_cli_history_pos < ((int32_t)s_usb_cli_history_count - 1)){
		s_usb_cli_history_pos++;
		slot = mesc_cli_history_slot((uint32_t)s_usb_cli_history_pos);
		mesc_cli_set_line(s_usb_cli_history[slot]);
	}else{
		s_usb_cli_history_pos = -1;
		s_usb_cli_history_draft[s_usb_cli_history_draft_len] = '\0';
		mesc_cli_set_line(s_usb_cli_history_draft);
	}

	mesc_cli_redraw_line(txlen);
}

void USB_CDC_Callback(uint8_t *buffer, uint32_t len){
	uint32_t i;
	char const *resp = NULL;
	uint32_t txlen = 0U;

	if(buffer == NULL || len == 0U){
		return;
	}

	for(i = 0U; i < len; i++){
		uint8_t c = buffer[i];

		if(s_usb_cli_esc_state == 1U){
			s_usb_cli_esc_state = (c == '[') ? 2U : 0U;
			continue;
		}

		if(s_usb_cli_esc_state == 2U){
			s_usb_cli_esc_state = 0U;
			if(c == 'A'){
				mesc_cli_history_up(&txlen);
				continue;
			}
			if(c == 'B'){
				mesc_cli_history_down(&txlen);
				continue;
			}
			if(c == 'D'){
				if(s_usb_cli_cursor > 0U){
					s_usb_cli_cursor--;
					mesc_cli_tx_append_str(&txlen, "\x1b[D");
				}
				continue;
			}
			if(c == 'C'){
				if(s_usb_cli_cursor < s_usb_cli_line_len){
					s_usb_cli_cursor++;
					mesc_cli_tx_append_str(&txlen, "\x1b[C");
				}
				continue;
			}
			if(c == 'H'){
				s_usb_cli_cursor = 0U;
				mesc_cli_redraw_line(&txlen);
				continue;
			}
			if(c == 'F'){
				s_usb_cli_cursor = s_usb_cli_line_len;
				mesc_cli_redraw_line(&txlen);
				continue;
			}
			if(c == '3'){
				s_usb_cli_esc_state = 3U;
				continue;
			}
			continue;
		}

		if(s_usb_cli_esc_state == 3U){
			s_usb_cli_esc_state = 0U;
			if(c == '~' && s_usb_cli_cursor < s_usb_cli_line_len){
				memmove(&s_usb_cli_line[s_usb_cli_cursor],
						&s_usb_cli_line[s_usb_cli_cursor + 1U],
						s_usb_cli_line_len - s_usb_cli_cursor - 1U);
				s_usb_cli_line_len--;
				mesc_cli_redraw_line(&txlen);
			}
			continue;
		}

		if(c == 0x1BU){
			s_usb_cli_esc_state = 1U;
			continue;
		}

		if(c == '\n' && s_usb_cli_skip_lf != 0U){
			s_usb_cli_skip_lf = 0U;
			continue;
		}

		if(c == '\r' || c == '\n'){
			s_usb_cli_skip_lf = (c == '\r') ? 1U : 0U;
			mesc_cli_tx_append_str(&txlen, "\r\n");
			if(s_usb_cli_line_len > 0U){
				s_usb_cli_line[s_usb_cli_line_len] = '\0';
				mesc_cli_store_history();
				if(mesc_cli_minimal_dispatch(s_usb_cli_line, s_usb_cli_resp, sizeof(s_usb_cli_resp))){
					resp = s_usb_cli_resp;
				}
				s_usb_cli_line_len = 0U;
				s_usb_cli_cursor = 0U;
			}
			s_usb_cli_history_pos = -1;
			s_usb_cli_history_draft_len = 0U;
			s_usb_cli_history_draft[0] = '\0';
			continue;
		}

		s_usb_cli_skip_lf = 0U;

		if((c == '\b') || (c == 0x7FU)){
			if(s_usb_cli_cursor > 0U){
				memmove(&s_usb_cli_line[s_usb_cli_cursor - 1U],
						&s_usb_cli_line[s_usb_cli_cursor],
						s_usb_cli_line_len - s_usb_cli_cursor);
				s_usb_cli_line_len--;
				s_usb_cli_cursor--;
				mesc_cli_redraw_line(&txlen);
			}
			continue;
		}

		if((c >= 32U) && (c <= 126U)){
			if(s_usb_cli_line_len < (USB_CLI_LINE_MAX - 1U)){
				if(s_usb_cli_cursor < s_usb_cli_line_len){
					memmove(&s_usb_cli_line[s_usb_cli_cursor + 1U],
							&s_usb_cli_line[s_usb_cli_cursor],
							s_usb_cli_line_len - s_usb_cli_cursor);
				}
				s_usb_cli_line[s_usb_cli_cursor] = (char)c;
				s_usb_cli_line_len++;
				s_usb_cli_cursor++;
				mesc_cli_redraw_line(&txlen);
			}else{
				s_usb_cli_line_len = 0U;
				s_usb_cli_cursor = 0U;
				resp = "ERR OVERFLOW\r\n";
			}
		}
	}

	if(resp != NULL){
		uint32_t rlen = (uint32_t)strlen(resp);
		uint32_t j;
		for(j = 0U; j < rlen && txlen < sizeof(s_usb_cli_tx); j++){
			s_usb_cli_tx[txlen++] = (uint8_t)resp[j];
		}
	}

	if(txlen > 0U){
		if(txlen > 0xFFFFU){
			txlen = 0xFFFFU;
		}
		(void)CDC_Transmit_FS(s_usb_cli_tx, (uint16_t)txlen);
	}
}

#ifdef HAL_CAN_MODULE_ENABLED
TASK_CAN_handle can1 = {0};
#endif

void populate_vars(void);

static bool mesc_vars_populated = false;
static bool mesc_persist_loaded = false;
static bool mesc_commands_registered = false;

static TermVariableHandle s_varHandle;

static bool mesc_prepare_var_system(void){
	if(null_handle.varHandle == NULL){
		memset(&s_varHandle, 0, sizeof(s_varHandle));
		null_handle.varHandle = &s_varHandle;
		null_handle.varHandle->varListHead = &TERM_varList;
	}else if(null_handle.varHandle->varListHead == NULL){
		null_handle.varHandle->varListHead = &TERM_varList;
	}

	if(TERM_VAR_init(&null_handle,
					 (uint8_t*)RTOS_flash_base_address(),
					 RTOS_flash_base_size(),
					 RTOS_flash_clear,
					 RTOS_flash_start_write,
					 RTOS_flash_write,
					 RTOS_flash_end_write) == NULL){
		return false;
	}

	return true;
}

static TermVariableDescriptor * mesc_find_var_by_name(TermVariableDescriptor *head, char const *name){
	TermVariableDescriptor *curr;

	if(head == NULL || name == NULL){
		return NULL;
	}

	curr = head->nextVar;
	while(curr != NULL){
		if(strcmp(curr->name, name) == 0){
			return curr;
		}
		curr = curr->nextVar;
	}

	return NULL;
}

static bool mesc_apply_persist_reader(TERMINAL_HANDLE *handle){
	MESC_PersistDataset dataset;
	MESC_PersistEntry entry;
	TermVariableDescriptor *head;
	uint32_t i;
	uint32_t missing_name = 0;
	uint32_t type_mismatch = 0;
	uint32_t parse_errors = 0;
	uint32_t rw_checked = 0;
	uint32_t rw_matches = 0;
	uint32_t rw_mismatches = 0;
	uint32_t ro_skipped = 0;
	bool load_ok = true;

	if(handle == NULL || handle->varHandle == NULL || handle->varHandle->varListHead == NULL){
		return false;
	}

	head = handle->varHandle->varListHead;

	if(!MESC_PersistOpenLatest(&dataset, (void const *)RTOS_flash_base_address(), RTOS_flash_base_size())){
		ttprintf("[persist-load] no dataset found in NVM\r\n");
		return false;
	}

	if(!MESC_PersistValidate(&dataset)){
		ttprintf("[persist-load] dataset validation failed (rev=%lu entries=%lu)\r\n",
				 (unsigned long)dataset.revision,
				 (unsigned long)dataset.entry_count);
		return false;
	}

	for(i = 0; i < dataset.entry_count; i++){
		TermVariableDescriptor *currVar;

		if(!MESC_PersistGetEntry(&dataset, i, &entry)){
			parse_errors++;
			load_ok = false;
			continue;
		}

		currVar = mesc_find_var_by_name(head, entry.name);
		if(currVar == NULL){
			missing_name++;
			continue;
		}

		if(((uint32_t)currVar->type != entry.type) || (currVar->typeSize != entry.type_size)){
			type_mismatch++;
			load_ok = false;
			continue;
		}

		if((currVar->rw & VAR_ACCESS_W) != 0U){
			rw_checked++;
			if(memcmp(currVar->variable, entry.value, currVar->typeSize) == 0){
				rw_matches++;
			}else{
				memcpy(currVar->variable, entry.value, currVar->typeSize);
				rw_mismatches++;
			}
		}else{
			ro_skipped++;
		}
	}

	ttprintf("[persist-load] rev=%lu entries=%lu parse_err=%lu missing=%lu type_mismatch=%lu rw_checked=%lu rw_match=%lu rw_applied=%lu ro_skipped=%lu\r\n",
			 (unsigned long)dataset.revision,
			 (unsigned long)dataset.entry_count,
			 (unsigned long)parse_errors,
			 (unsigned long)missing_name,
			 (unsigned long)type_mismatch,
			 (unsigned long)rw_checked,
			 (unsigned long)rw_matches,
			 (unsigned long)rw_mismatches,
			 (unsigned long)ro_skipped);

	if(parse_errors != 0U || type_mismatch != 0U){
		load_ok = false;
	}

	return load_ok;
}

void MESCinterface_startup_init(void){
	if(mesc_persist_loaded){
		return;
	}

	if(!mesc_prepare_var_system()){
		for(int i = 0; i < NUM_MOTORS; i++){
			mtr[i].conf_is_valid = false;
		}
		return;
	}

	if(!mesc_vars_populated){
		populate_vars();
		mesc_vars_populated = true;
	}

	if(!mesc_apply_persist_reader(&null_handle)){
		if(CMD_varLoad(&null_handle, 0, NULL) == TERM_CMD_EXIT_ERROR){
			for(int i = 0; i < NUM_MOTORS; i++){
				mtr[i].conf_is_valid = false;
			}
		}
	}

	calculateGains(&mtr[0]);
	calculateVoltageGain(&mtr[0]);
	calculateFlux(&mtr[0]);
	MESCinput_Init(&mtr[0]);

	mesc_persist_loaded = true;
}

void handleEscape(TERMINAL_HANDLE *handle){
	MESC_motor_typedef * motor_curr = &mtr[0];
	motor_curr->input_vars.UART_req = 0;
	motor_curr->input_vars.UART_dreq = 0;
}

const char TERM_startupText1[] = "\r\n";
const char TERM_startupText2[] = "\r\n[M]olony CHANGED2 [E]lectronic [S]peed [C]ontroller";
const char TERM_startupText3[] = "\r\n";

uint8_t CMD_error(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	bool has_error=false;
	uint32_t mask = 0;
	for(uint8_t i=0;i<32;i++){
		mask = (1 << i);
		if(MESC_errors & mask){
			ttprintf("Error bit[%u]: %s\r\n", i+1, error_string[i]);
			has_error = true;
		}
	}
	if(has_error == false){
		ttprintf("No errors :)\r\n");
	}
	return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_measure(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	MESC_motor_typedef * motor_curr = &mtr[0];

	bool measure_RL = false;
	bool measure_res = false;
	bool measure_ind = false;
	bool measure_kv  = false;
	bool measure_linkage = false;
	bool measure_hfi = false;
	bool measure_dt = false;

	if(argCount==0){
		measure_RL = true;
		measure_kv = true;
	}

	for(int i=0;i<argCount;i++){
		if(strcmp(args[i], "-a")==0){
			measure_RL = true;
			measure_kv = true;
		}
		if(strcmp(args[i], "-r")==0){
			measure_RL = true;
		}
		if(strcmp(args[i], "-s")==0){
			measure_res = true;
		}
		if(strcmp(args[i], "-l")==0){
			measure_ind = true;
		}
		if(strcmp(args[i], "-h")==0){
			measure_hfi = true;
		}
		if(strcmp(args[i], "-f")==0){
			measure_kv = true;
		}
		if(strcmp(args[i], "-g")==0){
			measure_linkage = true;
		}
		if(strcmp(args[i], "-d")==0){
			measure_dt = true;
		}
		if(strcmp(args[i], "-?")==0){
			ttprintf("Usage: measure [flags]\r\n");
			ttprintf("Ensure you set the measure current and voltage below the max voltage\r\n");
			ttprintf("\t -a\t Measure all\r\n");
			ttprintf("\t -r\t Measure resistance and inductance\r\n");
			ttprintf("\t -f\t Measure flux linkage\r\n");
			ttprintf("\t -g\t Measure flux linkage threshold v2\r\n");
			ttprintf("\t -h\t Measure HFI threshold\r\n");
			ttprintf("\t -d\t Measure deadtime compensation\r\n");
			ttprintf("\t -c\t Specify openloop current\r\n");
			ttprintf("\t -v\t Specify HFI voltage\r\n");
			return TERM_CMD_EXIT_SUCCESS;
		}
		if(strcmp(args[i], "-v")==0){
			if(i+1 < argCount){
				motor.measure_voltage = strtof(args[i+1], NULL);
			}
		}
		if(strcmp(args[i], "-c")==0){
			if(i+1 < argCount){
				motor.measure_current = strtof(args[i+1], NULL);
			}
		}
	}

	if(measure_RL){
		//Measure resistance and inductance
		motor_curr->MotorState = MOTOR_STATE_MEASURING;
		ttprintf("Measuring resistance and inductance\r\nWaiting for result");
		mtr[0].meas.PWM_cycles = 0;

		while(motor_curr->MotorState == MOTOR_STATE_MEASURING){
			HAL_Delay(200);
			ttprintf(".");
		}

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);

		float R, Lq, Ld;
		char* Runit;
		char* Lunit;
		if(motor_curr->m.R > 0){
			R = motor_curr->m.R;
			Runit = "Ohm";
		}else{
			R = motor_curr->m.R*1000.0f;
			Runit = "mOhm";
		}
		if(motor_curr->m.L_Q > 0.001f){
			Ld = motor_curr->m.L_D*1000.0f;
			Lq = motor_curr->m.L_Q*1000.0f;
			Lunit = "mH";
		}else{
			Ld = motor_curr->m.L_D*1000.0f*1000.0f;
			Lq = motor_curr->m.L_Q*1000.0f*1000.0f;
			Lunit = "uH";
		}


		ttprintf("R = %f %s\r\nLd = %f %s\r\nLq = %f %s\r\n\r\n", (double)R, Runit, (double)Ld, Lunit, (double)Lq, Lunit);
		calculateGains(motor_curr);
		HAL_Delay(1000);
	}

	if(measure_res){
		//Measure resistance
		float old_L_D = motor_curr->m.L_D;
		motor_curr->m.R = 0.0001f;//0.1mohm, really low
		motor_curr->m.L_D = 0.000001f;//1uH, really low
		calculateGains(motor_curr);
		calculateVoltageGain(motor_curr);

		motor_curr->MotorState = MOTOR_STATE_RUN;
		motor_curr->MotorSensorMode = MOTOR_SENSOR_MODE_OPENLOOP;

		motor_curr->FOC.openloop_step = 0;

		ttprintf("Measuring resistance \r\nWaiting for result");
		int a=200;
		float Itop = 0.0f;
		float Ibot = 0.0f;
		float Vtop = 0.0f;
		float Vbot = 0.0f;
		motor_curr->input_vars.UART_req = 0.45f*motor_curr->m.Imax;

		while(a){
			HAL_Delay(5);
			ttprintf(".");
			Ibot = Ibot+motor_curr->FOC.Idq.q;
			Vbot = Vbot+motor_curr->FOC.Vdq.q;
			a--;
			motor_curr->FOC.FOCAngle +=300;
		}
		a=200;
		motor_curr->input_vars.UART_req = 0.55f*motor_curr->m.Imax;
		while(a){
			HAL_Delay(5);
			ttprintf(".");
			Itop = Itop+motor_curr->FOC.Idq.q;
			Vtop = Vtop+motor_curr->FOC.Vdq.q;
			a--;
			motor_curr->FOC.FOCAngle +=300;
		}

		motor_curr->m.R = (Vtop-Vbot)/((Itop-Ibot)); //Calculate the resistance

		motor_curr->input_vars.UART_req = 0.0f;
		motor_curr->MotorState = MOTOR_STATE_TRACKING;
		motor_curr->MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);

		float R;
		char* Runit;
		if(motor_curr->m.R > 0){
			R = motor_curr->m.R;
			Runit = "Ohm";
		}else{
			R = motor_curr->m.R*1000.0f;
			Runit = "mOhm";
		}


		ttprintf("R = %f %s\r\n\r\n", (double)R, Runit);
		motor_curr->m.L_D =old_L_D;
		calculateGains(motor_curr);
		HAL_Delay(1000);
	}

	if(measure_ind){
		//Measure inductance, second method

		ttprintf("Measuring Inductance\r\nWaiting for result");
		int a=200;
		float Loffset[3];
		float Lqoffset[3];


		//set things up to do the L measurement
		motor_curr->MotorState = MOTOR_STATE_RUN;
		motor_curr->input_vars.UART_req = 0.25f; //Stop it going into tracking mode
		motor_curr->HFI.Type = HFI_TYPE_SPECIAL;
		motor_curr->HFI.special_injectionVd = 0.2f;
		motor_curr->HFI.special_injectionVq = 0.0f;
		motor_curr->MotorSensorMode = MOTOR_SENSOR_MODE_OPENLOOP;
		motor_curr->FOC.openloop_step = 0;
		motor_curr->FOC.FOCAngle = 0;
		motor_curr->input_vars.UART_dreq = -5.0f;
		motor_curr->FOC.didq.d = 0.0f;
		HAL_Delay(10);

		//Determine the voltage required
		while(a){
			if(fabsf(motor_curr->FOC.didq.d)<5.0f){
				motor_curr->HFI.special_injectionVd *=1.05f;
				if(motor_curr->HFI.special_injectionVd > (0.5f * motor_curr->Conv.Vbus))
				{
					motor_curr->HFI.special_injectionVd = 0.5f * motor_curr->Conv.Vbus;
				}
			}
			HAL_Delay(5);
			ttprintf(".");
			a--;
		}
		int b;
		//Measure the Ld
		for(b=0;b<3; b++){
			Loffset[b] = 0.0f;
			a=200;
			motor_curr->input_vars.UART_dreq = -motor_curr->m.Imax * 0.25f * (float)b;
			while(a){
				Loffset[b] = Loffset[b] + motor_curr->FOC.didq.d;
				HAL_Delay(5);
				ttprintf(".");
				a--;
			}
		Loffset[b] = Loffset[b]/200;
		Loffset[b] = motor_curr->FOC.pwm_period * motor_curr->HFI.special_injectionVd/Loffset[b];
		}

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);
		ttprintf("D-Inductance = %f , %f , %f  H\r\n voltage was %f \r\n", (double)Loffset[0], (double)Loffset[1], (double)Loffset[2], (double)motor_curr->HFI.special_injectionVd);

		//Now do Lq
		motor_curr->HFI.special_injectionVq = motor_curr->HFI.special_injectionVd;
		float c = motor_curr->HFI.special_injectionVq;
		motor_curr->HFI.special_injectionVd = 0.0f;
		for(b=0;b<3; b++){
			Lqoffset[b] = 0.0f;
			a=200;
			motor_curr->input_vars.UART_dreq = -10.0f ;
			motor_curr->HFI.special_injectionVq = c * (1+(float)b);
			if(motor_curr->HFI.special_injectionVq > motor_curr->Conv.Vbus*0.5f){motor_curr->HFI.special_injectionVq = motor_curr->Conv.Vbus*0.5f;}
			while(a){
				Lqoffset[b] = Lqoffset[b] + motor_curr->FOC.didq.q;
				HAL_Delay(5);
				ttprintf(".");
				a--;
			}
		Lqoffset[b] = Lqoffset[b]/200;
		Lqoffset[b] = motor_curr->FOC.pwm_period * motor_curr->HFI.special_injectionVq/Lqoffset[b];
		}

		//Put things back to runable
		motor_curr->HFI.Type = HFI_TYPE_NONE;
		motor_curr->MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
		motor_curr->input_vars.UART_req = 0.0f; //Turn it off.
		motor_curr->input_vars.UART_dreq = 0.0f;
		motor_curr->MotorState = MOTOR_STATE_TRACKING;


		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);
		ttprintf("Q-Inductance = %f , %f , %f  H\r\n voltage was %f \r\n", (double)Lqoffset[0], (double)Lqoffset[1], (double)Lqoffset[2], (double)motor_curr->HFI.special_injectionVq);

		motor_curr->HFI.special_injectionVd = 0.0f;
		motor_curr->HFI.special_injectionVq = 0.0f;

		HAL_Delay(200);
	}


	if(measure_kv){
		//Measure kV

		motor_curr->MotorState = MOTOR_STATE_GET_KV;
		ttprintf("Measuring flux linkage\r\nWaiting for result");

		while(motor_curr->MotorState == MOTOR_STATE_GET_KV){
			HAL_Delay(200);
			ttprintf(".");
		}

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);

		ttprintf("Flux linkage = %f mWb\r\n\r\n", (double)(motor_curr->m.flux_linkage * 1000.0f));
		HAL_Delay(2000);
	}

	if(measure_linkage){
		//Measure kV
		motor_curr->MotorState = MOTOR_STATE_RUN;
		ttprintf("Measuring flux linkage\r\nWaiting for result");
		motor_curr->m.flux_linkage_max = 0.1001f;//Start it low
		motor_curr->m.flux_linkage_min = 0.00005f;//Start it low

		motor_curr->HFI.Type = HFI_TYPE_NONE;
		motor_curr->FOC.FW_curr_max = 0.1f;
		motor_curr->input_vars.UART_req = 10.0f; //Parametise later, closedloop current

		while((motor_curr->m.flux_linkage_max > 0.0001f) && (motor_curr->FOC.eHz<100)){
			HAL_Delay(10);
			ttprintf(".");
			motor_curr->m.flux_linkage_max = motor_curr->m.flux_linkage_max*0.997f;// + 0.0001f;
			motor_curr->FOC.flux_a = motor_curr->FOC.flux_a	+ 0.01*motor_curr->FOC.flux_b;
			motor_curr->FOC.flux_b = motor_curr->FOC.flux_b	- 0.01*motor_curr->FOC.flux_a;//Since the two fluxes are derivatives of each other, this kicks them around and avoids stalls
			if(motor_curr->MotorState == MOTOR_STATE_ERROR){
				break;
			}
		}
		int a=200;
		motor_curr->m.flux_linkage_max = motor_curr->m.flux_linkage_max*1.5f;
		while(a){
			HAL_Delay(10);
			ttprintf(".");
			a--;
		}
		motor_curr->m.flux_linkage_max = motor_curr->FOC.flux_observed*1.5f;
		motor_curr->m.flux_linkage_min = motor_curr->FOC.flux_observed*0.5f;
		motor_curr->m.flux_linkage = motor_curr->FOC.flux_observed;

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);

		ttprintf("Flux linkage = %f mWb\r\n\r\n", (double)(motor_curr->m.flux_linkage * 1000.0f));
		ttprintf("Did the motor spin for >2seconds?");

		HAL_Delay(200);

		motor_curr->input_vars.UART_req = 0.0f;
		motor_curr->MotorState = MOTOR_STATE_TRACKING;
	}

	if(measure_hfi){
		ttprintf("Measuring HFI threshold\r\n");
		float HFI_Threshold = MESCmeasure_DetectHFI(motor_curr);

		ttprintf("HFI threshold: %f\r\n", (double)HFI_Threshold);

		HAL_Delay(500);
	}

	if(measure_dt){
		ttprintf("Measuring deadtime compensation\r\nWaiting for result");
		motor_curr->MotorState = MOTOR_STATE_TEST;
		while(motor_curr->MotorState == MOTOR_STATE_TEST){
			HAL_Delay(200);
			ttprintf(".");
		}

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);

		ttprintf("Deadtime register: %d\r\n", motor_curr->FOC.deadtime_comp);
		HAL_Delay(500);
	}


    return TERM_CMD_EXIT_SUCCESS;
}


void callback(TermVariableDescriptor * var){
	calculateFlux(&mtr[0]);
	calculateGains(&mtr[0]);
	calculateVoltageGain(&mtr[0]);
	MESCinput_Init(&mtr[0]);
}

void populate_vars(){
	//		   | Variable							| MIN		| MAX		| NAME			| DESCRIPTION																				| RW			| CALLBACK	| VAR LIST HANDLE
	TERM_addVar(mtr[0].m.Pmax						, 0.0f		, 50000.0f	, "par_p_max"	, "Max power"																				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.IBatmax					, 0.0f		, 1000.0f	, "par_ibat_max", "Max battery current power"																				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.direction					, 0			, 1			, "par_dir"		, "Motor direction"																			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.pole_pairs					, 0			, 255		, "par_pp"		, "Motor pole pairs"																		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.RPMmax						, 0			, 300000	, "par_rpm_max"	, "Max RPM"																					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.flux_linkage				, 0.0f		, 100.0f	, "par_flux"	, "Flux linkage"																			, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].m.flux_linkage_gain			, 0.0f		, 100.0f	, "FOC_flux_gain"	, "Flux linkage gain"																	, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.non_linear_centering_gain	, 0.0f		, 10000.0f	, "FOC_flux_nlin"	, "Flux centering gain"																	, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.flux_linkage_gain			, 0.0f		, 100.0f	, "FOC_flux_gain"	, "Flux linkage gain"																	, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.ortega_gain				, 1.0f		, 100000000.0f	, "FOC_ortega_gain"	, "Ortega gain, typically 1M"														, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].options.observer_type		, 0			, 4			, "FOC_obs_type", "Observer type, 0=None, 1=MXLLambda, 2MXL, 3=OrtegaOrig, 4=PLL"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.R							, 0.0f		, 10.0f		, "par_r"		, "Phase resistance"																		, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].m.L_D						, 0.0f		, 10.0f		, "par_ld"		, "Phase inductance"																		, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].m.L_Q						, 0.0f		, 10.0f		, "par_lq"		, "Phase inductance"																		, VAR_ACCESS_RW	, callback  , &TERM_varList);
	TERM_addVar(mtr[0].FOC.Current_bandwidth		, 200.0f	, 10000.0f	, "FOC_curr_BW"	, "Current Controller Bandwidth"															, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].HFI.Type						, 0			, 3			, "FOC_hfi_type"	, "HFI type [0=None, 1=45deg, 2=d axis]"												, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].meas.hfi_voltage				, 0.0f		, 50.0f		, "FOC_hfi_volt"	, "HFI voltage"																			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	//TERM_addVar(mtr[0].HFI.mod_didq					, 0.0f		, 2.0f		, "FOC_hfi_gain"	, "HFI gain"																			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].HFI.toggle_eHz				, 0.0f		, 2000.0f	, "FOC_hfi_eHz"	, "HFI Max Frequency"																		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.FW_curr_max				, 0.0f		, 300.0f	, "par_fw_curr"		, "Max field weakenning current"														, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].meas.measure_current			, 0.5f		, 100.0f	, "meas_curr"	, "Measuring current"																		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].meas.measure_closedloop_current, 0.5f	, 100.0f	, "meas_cl_curr", "Measuring q closed loop current"															, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].meas.measure_voltage			, 0.5f		, 100.0f	, "meas_volt"	, "Measuring voltage"																		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.adc1_MAX			, 0			, 4096		, "adc1_max"	, "ADC1 max val"																			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.adc1_MIN			, 0			, 4096		, "adc1_min"	, "ADC1 min val"																			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.ADC1_polarity		, -1.0f		, 1.0f		, "adc1_pol"	, "ADC1 polarity"																			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.adc2_MAX			, 0			, 4096		, "adc2_max"	, "ADC2 max val"																			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.adc2_MIN			, 0			, 4096		, "adc2_min"	, "ADC2 min val"																			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.ADC2_polarity		, -1.0f		, 1.0f		, "adc2_pol"	, "ADC2 polarity"																			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.max_request_Idq.q	, 0.0f		, 1000.0f	, "par_i_max"	, "Max motor current"																		, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.min_request_Idq.q	, -1000.0f	, 0.0f		, "par_i_min"	, "Min motor current"																		, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.pwm_frequency			, 0.0f		, 100000.0f	, "FOC_fpwm"	, "PWM frequency"																			, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.Modulation_max			, 0.1f		, 1.12f	, 	"FOC_Max_Mod"	, "Max modulation index; typically 0.95, can over modulate to 1.12"							, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.UART_req			, -1000.0f	, 1000.0f	, "uart_req"	, "Uart input"																				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.UART_dreq			, -1000.0f	, 1000.0f	, "uart_dreq"	, "Uart input"																				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.input_options		, 0			, 128		, "input_opt"	, "Inputs [1=ADC1 2=ADC2 4=PPM 8=UART 16=Killswitch 32=CANADC1 64=CANADC2 128=ADC12DIFF]"	, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].safe_start[0]				, 0			, 1000		, "safe_start"	, "Countdown before allowing throttle"														, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].safe_start[1]				, 0			, 1000		, "safe_count"	, "Live count before allowing throttle"														, VAR_ACCESS_R	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.enc_offset				, 0			, 65535		, "FOC_enc_oset", "Encoder alignment angle"																	, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.FOCAngle					, 0			, 65535		, "FOC_angle"	, "FOC angle now"																			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.enc_angle				, 0			, 65535		, "FOC_enc_ang"	, "Encoder angle now"																		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.enc_counts					, 0			, 65535		, "FOC_enc_PPR"	, "Encoder ABI PPR"																			, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.encoder_polarity_invert	, 0			, 1			, "FOC_enc_pol"	, "Encoder polarity"																		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].MotorSensorMode				, 0			, 30		, "par_motor_sensor", "0=SL, 1=Hall, 2=OL, 3=ABSENC, 4=INC_ENC, 5=HFI"										, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].SLStartupSensor				, 0			, 30		, "par_SL_sensor"	, "0=OL, 1=Hall, 2=PWMENC, 3=HFI"														, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.openloop_step			, 0.0f		, 6000.0f	, "FOC_ol_step"	, "Angle per PWM period openloop (65535 per erev)"											, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.FW_ehz_max				, 0.0f		, 6000.0f	, "FOC_fw_ehz"	, "max eHz under field weakenning"															, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.park_current				, 0.0f		, 300.0f	, "par_i_park"	, "Max current for handbrake"																, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.hall_IIR					, 0.0f		, 1.0f		, "FOC_hall_iir", "Decay constant for hall preload (0-1.0)"													, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.hall_transition_V		, 0.0f		, 100.0f	, "FOC_hall_Vt"	, "Hall transition voltage"																	, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.hall_initialised			, 0			, 1			, "FOC_hall_array_ok"	, "Hall array OK flag (set to 0 to restart live hall cal process)"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(MESC_all_errors						, -HUGE_VAL	, HUGE_VAL	, "error_all"	, "All errors encountered"																	, VAR_ACCESS_R	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].options.field_weakening		, 0			, 2			, "opt_fw"		, "Field weakening [0=OFF, 1=ON, 2=ON V2]"													, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].options.sqrt_circle_lim		, 0			, 2			, "opt_circ_lim", "Circle limiter [0=OFF, 1=ON, 2=ON Vd]"													, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].options.pwm_type				, 0			, 3			, "opt_pwm_type", "Modulator [0=SVPWM, 1=sinusoidal, 2=Bottom clamp, 3=Sin/bottom combo]"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].options.MTPA_mode			, 0			, 3			, "opt_mtpa"	, "MTPA type = 0=none, 1=setpoint, 2=magnitude, 3=iq"										, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].options.use_hall_start		, 0			, 1			, "opt_hall_start", "Use hall start"																		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].options.use_phase_balancing	, 0			, 1			, "opt_phase_bal", "Use highhopes phase balancing"															, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].options.use_lr_observer		, 0			, 1			, "opt_lr_obs"  , "Use LR observer"																			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].options.has_motor_temp_sensor, 0			, 1			, "opt_motor_temp"  , "Motor has temperature sensor"														, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].options.app_type				, 0			, 3			, "opt_app_type"  , "App type, 0=none, 1=Vehicle, 2,3 = undefined"											, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].ControlMode					, 0			, 4			, "opt_cont_type"  , "Cont type: 0=Torque, 1=Speed, 2=Duty, 3=Position, 4=Measuring, 5=Handbrake"			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.FOC_advance				, -10.0f	, 10.0f		, "FOC_Advance"	, "FOC advance, proportion of 1 PWM cycle"													, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.speed_kp					, 0.0f		, 6000000.0f, "speed_kp"	, "amps/Hz proportional gain"																, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.speed_ki					, 0.0f		, 6000000.0f, "speed_ki"	, "amps/Hz integral gain"																	, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.speed_req				, 0.0f		, 5000.0f	, "speed_req"	, "Hz"																						, VAR_ACCESS_RW	, callback	, &TERM_varList);




//	_motor->FOC.FOC_advance

	TERM_addVarArrayFloat(mtr[0].m.hall_flux, sizeof(mtr[0].m.hall_flux),  -10.0f, 10.0f, "Hall_flux", "hall start table", VAR_ACCESS_RW, NULL, &TERM_varList);

	#ifdef HAL_CAN_MODULE_ENABLED
	TERM_addVar(can1.node_id						, 1			, 254		, "node_id"	    , "Node ID"																					, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].input_vars.remote_ADC_can_id	, 0			, 254		, "can_adc"	    , "CAN ADC ID  0=disabled"																	, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(can1.rx_dropped, 0, 4294967295U, "can_rx_drop", "CAN RX queue drops", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.stream_dropped, 0, 4294967295U, "can_stream_drop", "CAN terminal stream drops", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.rx_isr_frames, 0, 4294967295U, "can_rx_isr", "CAN RX ISR accepted frames", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.rx_isr_filtered, 0, 4294967295U, "can_rx_filt", "CAN RX ISR filtered frames", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.rx_isr_getmsg_err, 0, 4294967295U, "can_rx_err", "CAN RX ISR HAL get errors", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.rx_packets_handled, 0, 4294967295U, "can_rx_proc", "CAN RX task processed packets", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.tx_frames_sent, 0, 4294967295U, "can_tx_sent", "CAN TX HAL sent frames", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.tx_frames_failed, 0, 4294967295U, "can_tx_fail", "CAN TX HAL failed frames", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.tx_enqueue_dropped, 0, 4294967295U, "can_tx_qdrop", "CAN TX queue enqueue drops", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.tx_mailbox_full, 0, 4294967295U, "can_tx_mbox", "CAN TX mailbox busy while pending", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.posvel_sent, 0, 4294967295U, "can_posvel_sent", "CAN POSVEL frames sent", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.posvel_slot_miss, 0, 4294967295U, "can_posvel_miss", "CAN POSVEL slot misses", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.posvel_tx_blocked, 0, 4294967295U, "can_posvel_block", "CAN POSVEL TX blocked/fail", VAR_ACCESS_R, NULL, &TERM_varList);
	TERM_addVar(can1.posvel_max_gap_ticks, 0, 4294967295U, "can_posvel_gap", "CAN POSVEL max gap in OS ticks", VAR_ACCESS_R, NULL, &TERM_varList);
#endif

	TermVariableDescriptor * desc;
	desc = TERM_addVar(mtr[0].Conv.Vbus         	, 0.0f      , HUGE_VAL  , "vbus"        , "Read input voltage"                  													, VAR_ACCESS_TR , NULL      , &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

	desc = TERM_addVar(mtr[0].FOC.eHz           	, -HUGE_VAL , HUGE_VAL  , "ehz"         , "Motor electrical hz"                 													, VAR_ACCESS_TR , NULL      , &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

	desc = TERM_addVar(mtr[0].FOC.Idq_smoothed.d 	, -HUGE_VAL , HUGE_VAL  , "id"      	, "Phase Idq_d smoothed"                   													, VAR_ACCESS_TR , NULL      , &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

	desc = TERM_addVar(mtr[0].FOC.Idq_smoothed.q 	, -HUGE_VAL , HUGE_VAL  , "iq"      	, "Phase Idq_q smoothed"                   													, VAR_ACCESS_TR , NULL      , &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

	desc = TERM_addVar(mtr[0].Raw.ADC_in_ext1    	, 0			, 4096      , "adc1"   		, "Raw ADC throttle"                    													, VAR_ACCESS_TR , NULL      , &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

	desc = TERM_addVar(mtr[0].Conv.MOSu_T        	, 0.0f		, 4096.0f   , "TMOS"   		, "MOSFET temp, kelvin"                     												, VAR_ACCESS_TR , NULL      , &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

	desc = TERM_addVar(mtr[0].Conv.Motor_T       	, 0.0f 		, 4096.0f   , "TMOT"   		, "Motor temp, kelvin"                      												, VAR_ACCESS_TR , NULL      , &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

	desc = TERM_addVar(MESC_errors          		, -HUGE_VAL , HUGE_VAL  , "error" 		, "System errors now"       																, VAR_ACCESS_TR , NULL		, &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

	desc = TERM_addVar(mtr[0].FOC.Vdq.q     		, -4096.0f 	, 4096.0f  	, "Vq"    		, "FOC_Vdq_q"     																			, VAR_ACCESS_TR , NULL      , &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

	desc = TERM_addVar(mtr[0].FOC.Vdq.d     		, -4096.0f 	, 4096.0f  	, "Vd"    		, "FOC_Vdq_d"     																			, VAR_ACCESS_TR , NULL      , &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

	desc = TERM_addVar(mtr[0].FOC.Idq_req.q 		, -4096.0f 	, 4096.0f  	, "iqreq" 		, "mtr[0].FOC.Idq_req.q"     																, VAR_ACCESS_TR , NULL      , &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

}

#ifdef HAL_CAN_MODULE_ENABLED

#define REMOTE_ADC_TIMEOUT 1000

void TASK_CAN_packet_cb(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len){
	MESC_motor_typedef * motor_curr = &mtr[0];

	switch(id){
		case CAN_ID_IQREQ:{
			s_can_iqreq_rx_count++;
			if(sender == motor_curr->input_vars.remote_ADC_can_id && motor_curr->input_vars.remote_ADC_can_id > 0){
				volatile float req = mesc_unpack_float(data);
				if(req > 0.0f){
					// Owen addition
					motor_curr->input_vars.remote_ADC_timeout = REMOTE_ADC_TIMEOUT;
					// motor_curr->input_vars.UART_req = req * motor_curr->input_vars.max_request_Idq.q;
					motor_curr->input_vars.UART_req = req;
				}else{
					// Owen addition
					motor_curr->input_vars.remote_ADC_timeout = REMOTE_ADC_TIMEOUT;
					// motor_curr->input_vars.UART_req = req * motor_curr->input_vars.min_request_Idq.q;
					motor_curr->input_vars.UART_req = req;
				}
			}
			break;
		}
		case CAN_ID_SAMPLE_NOW:
			motor_curr->logging.sample_no_auto_send = true;
			motor_curr->logging.sample_now = true;
			break;
		case CAN_ID_SAMPLE_SEND:
			motor_curr->logging.sample_no_auto_send = false;
			break;
		case CAN_ID_ADC1_2_REQ:{
			if(sender == motor_curr->input_vars.remote_ADC_can_id && motor_curr->input_vars.remote_ADC_can_id > 0){
				motor_curr->input_vars.remote_ADC_timeout = REMOTE_ADC_TIMEOUT;
				motor_curr->input_vars.remote_ADC1_req = mesc_unpack_float(data);
				motor_curr->input_vars.remote_ADC2_req = mesc_unpack_float(data+4);
			}
			break;
		}
		default:
			break;
	}
}

void TASK_CAN_telemetry_fast(TASK_CAN_handle * handle){

	MESC_motor_typedef * motor_curr = &mtr[0];

	TASK_CAN_add_float(handle	, CAN_ID_ADC1_2_REQ	  	, CAN_BROADCAST, motor_curr->input_vars.ADC1_req		, motor_curr->input_vars.ADC2_req	, 0);
	TASK_CAN_add_float(handle	, CAN_ID_SPEED		  	, CAN_BROADCAST, motor_curr->FOC.eHz		, 0.0f					, 0);
	TASK_CAN_add_float(handle	, CAN_ID_BUS_VOLT_CURR 	, CAN_BROADCAST, motor_curr->Conv.Vbus		, motor_curr->FOC.Ibus	, 0);
	TASK_CAN_add_uint32(handle	, CAN_ID_STATUS	  		, CAN_BROADCAST, motor_curr->MotorState		, 0						, 0);
	TASK_CAN_add_float(handle	, CAN_ID_MOTOR_CURRENT 	, CAN_BROADCAST, motor_curr->FOC.Idq.q		, motor_curr->FOC.Idq.d	, 0);
	TASK_CAN_add_float(handle	, CAN_ID_MOTOR_VOLTAGE 	, CAN_BROADCAST, motor_curr->FOC.Vdq.q		, motor_curr->FOC.Vdq.d	, 0);

	// these values are somewhat untested in that I've
	//  never been able to make fastLoop() slow down
	uint32_t n       = motor_curr->jitter.samples;
    int32_t  min_cyc = motor_curr->jitter.min_cyc;
    int32_t  max_cyc = motor_curr->jitter.max_cyc;
    int64_t  sum_cyc = motor_curr->jitter.sum_cyc;

    if (n > 0 && min_cyc != INT32_MAX && max_cyc != INT32_MIN) {
        const float cyc2us = 1.0f / ((float)SystemCoreClock / 1e6f);
        motor_curr->jitter.avg_us = ((float)sum_cyc / (float)n) * cyc2us;
        motor_curr->jitter.p2p_us = (float)(max_cyc - min_cyc) * cyc2us;

        TASK_CAN_add_float(handle, CAN_ID_JITTER, CAN_BROADCAST, motor_curr->jitter.avg_us, motor_curr->jitter.p2p_us, 0);

        motor_curr->jitter.clear_req = 1; // ask ISR to reset window
    }

}

void TASK_CAN_telemetry_slow(TASK_CAN_handle * handle){
	MESC_motor_typedef * motor_curr = &mtr[0];

	TASK_CAN_add_float(handle	, CAN_ID_TEMP_MOT_MOS1	, CAN_BROADCAST, motor_curr->Conv.Motor_T			, motor_curr->Conv.MOSu_T			, 0);
	TASK_CAN_add_float(handle	, CAN_ID_TEMP_MOS2_MOS3	, CAN_BROADCAST, motor_curr->Conv.MOSv_T			, motor_curr->Conv.MOSw_T			, 0);
	TASK_CAN_add_uint32(handle	, CAN_ID_FOC_HYPER		, CAN_BROADCAST, motor_curr->FOC.cycles_fastloop	, motor_curr->FOC.cycles_pwmloop	, 0);

}

// We compute velocity here as Δθ/Δt from the encoder angle rather than using
// the FOC observer (PLL-based eHz) that MESC normally reports.
// The reason:
//  * PLL/observer velocity (default in MESC):
//      - Good at medium/high speed, tracks electrical frequency smoothly.
//      - Poor near standstill: noisy, can flip sign, lags true motion.
//      - Causes problems for balancing / precise low-speed control.
//  * Tick-timing (measuring Δt between encoder edges):
//      - Very accurate at very low speed.
//      - Becomes coarse/noisy at higher speeds and adds ISR jitter.
//  * Δθ/Δt method (this code):
//      - Simple, low-latency, and consistent across the full speed range.
//      - Works well for balancing robots where accurate small-angle velocity
//        is critical.
//      - Some quantization/noise at near-zero speed is expected (sign flip),
//        but easier to filter downstream than observer lag.
//
// Assumptions:

//  At standstill or low torque balance, the rotor
//  may jitter slightly forward/backward due to cogging,
//  quantization, or load disturbances. Each tiny tick
//  back-and-forth produces a velocity spike, and the sign flips every time.
// This must be dealt with on the controller side
//
// --- Encoder configuration ---
// Adjust ENCODER_CPR to match your hardware encoder resolution
// For a 12-bit ABI encoder (0–4095 counts per rev):
#define ENCODER_CPR 4096

// --- DWT cycle counter (hardware timer running at CPU frequency) ---
// Used to get precise microsecond timestamps.
// On STM32F405, core clock is 168 MHz → 168 cycles = 1 µs.
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)

// Persistent + debugger-visible state
static volatile float vel_est_filtered = 0.0f;
static volatile uint32_t last_pos = 0;
static volatile uint32_t last_time_us = 0;

// Debugger-visible intermediate values
static volatile float eHz_dbg = 0.0f;
static volatile float vel_PLL_dbg = 0.0f;
static volatile float vel_enc_dbg = 0.0f;
static volatile float vel_raw_dbg = 0.0f;
static volatile float dt_s_dbg = 0.0f;
static volatile int32_t delta_pos_dbg = 0;
static volatile uint32_t posvel_counter_dbg = 0U;
void TASK_CAN_telemetry_posvel(TASK_CAN_handle *handle) {
    MESC_motor_typedef *motor_curr = &mtr[0];
	uint32_t now_tick;
	uint32_t gap_tick;

    // === 1. Raw PLL output ===
    eHz_dbg = motor_curr->FOC.eHz;   // electrical Hz from FOC PLL

    // === 2. Convert PLL to mechanical rad/s ===
    vel_PLL_dbg = 0.0f;
    if (motor_curr->m.pole_pairs > 0) {
        vel_PLL_dbg = eHz_dbg * (2.0f * M_PI / (float)motor_curr->m.pole_pairs);
    }

    // === 3. Encoder-based velocity (Δθ/Δt) ===
    uint32_t pos_now = motor_curr->FOC.abs_position;  // absolute mechanical position in ticks

    // === 4. Read precise timestamp from DWT cycle counter ---
    uint32_t now_cycles = *DWT_CYCCNT;
    uint32_t time_now_us = now_cycles / 168;     // convert cycles → µs (for 168 MHz CPU)

    // === 5. Delta counts with rollover correction ---
    delta_pos_dbg = (int32_t)pos_now - (int32_t)last_pos;
    int32_t half_cpr = ENCODER_CPR / 2;
    if (delta_pos_dbg >  half_cpr) delta_pos_dbg -= ENCODER_CPR;
    if (delta_pos_dbg < -half_cpr) delta_pos_dbg += ENCODER_CPR;

    // === 6. Δt in seconds ---
    dt_s_dbg = (time_now_us - last_time_us) * 1e-6f;

    vel_enc_dbg = 0.0f;
    if (dt_s_dbg > 1e-6f) {
        float delta_rad = delta_pos_dbg * (2.0f * M_PI / (float)motor_curr->m.enc_counts);
        vel_enc_dbg = delta_rad / dt_s_dbg;  // mechanical rad/s
    }

    last_pos = pos_now;
    last_time_us = time_now_us;

    // === 7. Blend PLL and encoder velocities ===
    vel_raw_dbg = 0.0f;
    if (fabsf(vel_enc_dbg) < 5.0f) {
        vel_raw_dbg = 0.7f * vel_enc_dbg + 0.3f * vel_PLL_dbg;
    } else {
        vel_raw_dbg = 0.3f * vel_enc_dbg + 0.7f * vel_PLL_dbg;
    }

    // === 8. Low-pass filter ===
    const float alpha = 0.1f;  // smoothing factor
    vel_est_filtered = alpha * vel_raw_dbg + (1.0f - alpha) * vel_est_filtered;

	// === 9. Counter payload for POSVEL slot 0 ===
	float pos_counter = (float)posvel_counter_dbg;
	posvel_counter_dbg++;

    // === 10. if close to zero, force to zero ===
    if (fabsf(vel_est_filtered) < 0.02f) {   // threshold in rad/s
        vel_est_filtered = 0.0f;
    }

	// === 11. Send telemetry over CAN ===
	// Payload: counter [float-encoded], velocity [rad/s].
    // NOTE: vel_est_filtered is in radians per second.
    //       To get RPM, multiply by (60 / 2π).
	if(mesc_can_send_posvel_frame(handle, pos_counter, vel_est_filtered)){
		handle->posvel_sent++;

		now_tick = HAL_GetTick();
		gap_tick = now_tick - handle->posvel_last_tick;
		if(handle->posvel_last_tick != 0U && gap_tick > handle->posvel_max_gap_ticks){
			handle->posvel_max_gap_ticks = gap_tick;
		}

		handle->posvel_last_tick = now_tick;
    }else{
        handle->posvel_tx_blocked++;
	}

}

void MESCinterface_can_periodic(void){
	#ifdef HAL_CAN_MODULE_ENABLED
	static uint32_t last_tick_posvel = 0U;
	static uint32_t last_tick_status = 0U;
	uint32_t now_tick = HAL_GetTick();
	uint32_t period_tick_posvel = (1000U / POSVEL_HZ);
	if(period_tick_posvel == 0U) period_tick_posvel = 1U;
	const uint32_t period_tick_status = 10U; // 100Hz = 10ms

	mesc_can_hw_init_if_needed();
	if(!s_can_hw_ready){
		return;
	}

	// Poll and dispatch pending CAN frames in the known-active interface task.
	mesc_can_poll_rx_fifo(&can1);

	// POSVEL scheduler (500Hz by default)
	uint32_t elapsed_posvel = now_tick - last_tick_posvel;
	if(elapsed_posvel >= period_tick_posvel){
		if(last_tick_posvel != 0U){
			uint32_t slots_elapsed = elapsed_posvel / period_tick_posvel;
			if(slots_elapsed > 1U){
				can1.posvel_slot_miss += (slots_elapsed - 1U);
			}
		}
		last_tick_posvel = now_tick;
		TASK_CAN_telemetry_posvel(&can1);
	}

	// Motor status scheduler (100Hz)
	uint32_t elapsed_status = now_tick - last_tick_status;
	if(elapsed_status >= period_tick_status){
		last_tick_status = now_tick;
		TASK_CAN_telemetry_motor_status(&can1);
	}
	#endif
}
#define POST_ERROR_SAMPLES 		LOGLENGTH/2

void TASK_CAN_aux_data(TASK_CAN_handle * handle){
	static int samples_sent=-1;
	static int current_pos=0;
	static float timestamp;

	MESC_motor_typedef * motor_curr = &mtr[0];

	if(motor_curr->logging.print_samples_now && motor_curr->logging.sample_no_auto_send == false){
		if(samples_sent == -1){
			current_pos = motor_curr->logging.current_sample;
			TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, 0, 0, CAN_SAMPLE_FLAG_START, (float)LOGLENGTH, 100);
			samples_sent=0;
			timestamp = motor_curr->FOC.pwm_period * (float)POST_ERROR_SAMPLES * -1.0f;
			return;
		}

		timestamp += motor_curr->FOC.pwm_period;
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 0, 0, timestamp, 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 1, 0, motor_curr->logging.Vbus[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 2, 0, motor_curr->logging.Iu[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 3, 0, motor_curr->logging.Iv[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 4, 0, motor_curr->logging.Iw[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 5, 0, motor_curr->logging.Vd[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 6, 0, motor_curr->logging.Vq[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 7, 0, motor_curr->logging.angle[current_pos], 100);

		samples_sent++;
		current_pos++;
		if(current_pos == LOGLENGTH){
			current_pos = 0;
		}
		if(samples_sent == LOGLENGTH){
			timestamp = 0;
			samples_sent = -2;
			motor_curr->logging.print_samples_now = 0;
			motor_curr->logging.lognow = 1;
			return;
		}
	}
	if(samples_sent == -2){
		samples_sent = -1;
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, 0, 0, CAN_SAMPLE_FLAG_END, 0.0f, 100);
	}

}
#endif


void MESCinterface_init(TERMINAL_HANDLE * handle){
	if(handle == NULL){
		handle = &null_handle;
	}

	MESCinterface_startup_init();

	if(mesc_commands_registered){
		return;
	}
	mesc_commands_registered = true;

	TERM_addCommand(CMD_measure, "measure", "Measure motor R+L", 0, &TERM_defaultList);
	TERM_addCommand(CMD_error, "error", "Show errors", 0, &TERM_defaultList);

	(void)handle;

}
