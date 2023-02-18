/*
 **
 ******************************************************************************
 * @file           : hfi.c
 * @brief          : HFI debug task
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

#include "can.h"
#include "string.h"
#include "task_cli.h"
#include "init.h"

#define APP_NAME "can"
#define APP_DESCRIPTION "CAN test"
#define APP_STACK 512

static uint8_t CMD_main(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
static void TASK_main(void *pvParameters);
static uint8_t INPUT_handler(TERMINAL_HANDLE * handle, uint16_t c);


uint8_t REGISTER_can(TermCommandDescriptor * desc){
    TERM_addCommand(CMD_main, APP_NAME, APP_DESCRIPTION, 0, desc); 
    return pdTRUE;
}

static uint8_t CMD_main(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    
    uint8_t currArg = 0;
    uint8_t returnCode = TERM_CMD_EXIT_SUCCESS;
    char ** cpy_args=NULL;
    argCount++;
    if(argCount){
        cpy_args = pvPortMalloc(sizeof(char*)*argCount);
        cpy_args[0] = pvPortMalloc(sizeof(APP_NAME));
        cpy_args[0]=memcpy(cpy_args[0], APP_NAME, sizeof(APP_NAME));
        for(;currArg<argCount-1; currArg++){
            uint16_t len = strlen(args[currArg])+1;
            cpy_args[currArg+1] = pvPortMalloc(len);
            memcpy(cpy_args[currArg+1], args[currArg], len);
        }
    }
    TermProgram * prog = pvPortMalloc(sizeof(TermProgram));
    prog->inputHandler = INPUT_handler;
    prog->args = cpy_args;
    prog->argCount = argCount;
    TERM_sendVT100Code(handle, _VT100_RESET, 0); TERM_sendVT100Code(handle, _VT100_CURSOR_POS1, 0);
    returnCode = xTaskCreate(TASK_main, APP_NAME, APP_STACK, handle, tskIDLE_PRIORITY + 1, &prog->task) ? TERM_CMD_EXIT_PROC_STARTED : TERM_CMD_EXIT_ERROR;
    if(returnCode == TERM_CMD_EXIT_PROC_STARTED) TERM_attachProgramm(handle, prog);
    return returnCode;
}



void putbuffer_can_db(unsigned char *buf, unsigned int len, port_str * port, uint32_t id){
#ifdef HAL_CAN_MODULE_ENABLED
	xSemaphoreTake(port->tx_semaphore, portMAX_DELAY);
	while(len){
		uint32_t TxMailbox;
		if(HAL_CAN_GetTxMailboxesFreeLevel(port->hw)){
			uint32_t transmit_bytes = len>8 ? 8 : len;

			CAN_TxHeaderTypeDef TxHeader;
			TxHeader.StdId = id;
			TxHeader.ExtId = 0x01;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.DLC = transmit_bytes;
			TxHeader.TransmitGlobalTime = DISABLE;

			HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(port->hw, &TxHeader, buf, &TxMailbox);  //function to add message for transmition
			if(ret== HAL_OK){
				len -= transmit_bytes;
				buf += transmit_bytes;
			}
		}

		vTaskDelay(1);
	}
	xSemaphoreGive(port->tx_semaphore);
#endif
}


extern port_str main_can;

static void TASK_main(void *pvParameters){


    TERMINAL_HANDLE * handle = (TERMINAL_HANDLE*)pvParameters;
    port_str * port = handle->port;
#ifdef HAL_CAN_MODULE_ENABLED

    xSemaphoreTake(main_can.term_block, portMAX_DELAY);
    uint16_t c=0;
    uint8_t s_c;
    uint32_t id=0;
    uint8_t * ptr = "\r\n";

    if(handle->currProgram->argCount>1){
    	id = strtoul(handle->currProgram->args[1], NULL, 10);
    }

    putbuffer_can_db(ptr, 2, &main_can, id);


    do{

    	if(c!=CTRL_C && c!=0){
    		s_c = c;
    		putbuffer_can_db(&s_c, 1, &main_can, id);

    	}

    	if(HAL_CAN_GetRxFifoFillLevel(main_can.hw, CAN_RX_FIFO0)){
			xSemaphoreTake(port->term_block, portMAX_DELAY);
			CAN_RxHeaderTypeDef pheader;
			uint8_t buffer[8];
			HAL_CAN_GetRxMessage(main_can.hw, CAN_RX_FIFO0, &pheader, buffer);
			if(pheader.StdId == id){
				ttprintf(NULL, buffer, pheader.DLC);
			}
			xSemaphoreGive(port->term_block);
		}

        c=0;
        xStreamBufferReceive(handle->currProgram->inputStream,&c,sizeof(c),pdMS_TO_TICKS(1));
    }while(c!=CTRL_C);
    xSemaphoreGive(main_can.term_block);
#endif
    TERM_sendVT100Code(handle,_VT100_CURSOR_ENABLE, 0);
    TERM_killProgramm(handle);
}

static uint8_t INPUT_handler(TERMINAL_HANDLE * handle, uint16_t c){
    if(handle->currProgram->inputStream==NULL) return TERM_CMD_EXIT_SUCCESS;
  	xStreamBufferSend(handle->currProgram->inputStream,&c,2,20);
  	return TERM_CMD_CONTINUE;
}
