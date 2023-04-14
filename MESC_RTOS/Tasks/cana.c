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

#include "cana.h"
#include "string.h"
#include "task_cli.h"
#include "init.h"
#include "stdlib.h"

#define APP_NAME "can"
#define APP_DESCRIPTION "CAN test"
#define APP_STACK 512
#define RAW_INPUT 1

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
    prog->raw_input = 1;
    TERM_sendVT100Code(handle, _VT100_RESET, 0); TERM_sendVT100Code(handle, _VT100_CURSOR_POS1, 0);
    returnCode = xTaskCreate(TASK_main, APP_NAME, APP_STACK, handle, tskIDLE_PRIORITY + 1, &prog->task) ? TERM_CMD_EXIT_PROC_STARTED : TERM_CMD_EXIT_ERROR;
    if(returnCode == TERM_CMD_EXIT_PROC_STARTED) TERM_attachProgramm(handle, prog);
    return returnCode;
}



extern port_str main_can;

const char reset[] = "\r\ncls\r\n";
const char ctrl_text[] = "Press CTRL+C again to exit";

static void TASK_main(void *pvParameters){


    TERMINAL_HANDLE * handle = (TERMINAL_HANDLE*)pvParameters;
    port_str * port = handle->port;
#ifdef HAL_CAN_MODULE_ENABLED

    xSemaphoreTake(main_can.term_block, portMAX_DELAY);
    uint8_t c=0;
    uint32_t id=0;

    ttprintf("Press 2x CTRL+C to exit.\r\n");

    if(handle->currProgram->argCount>1){
    	id = strtoul(handle->currProgram->args[1], NULL, 10);
    }


    while(TASK_CAN_connect(&can1,id, 1)==0){
    	vTaskDelay(1);
    }
    int8_t ctrl_cnt = 2;


    xSemaphoreTake(main_can.tx_semaphore, portMAX_DELAY);

    xStreamBufferSend(main_can.tx_stream, &reset, sizeof(reset)-1, 100);

    uint32_t last_ping = xTaskGetTickCount();

    do{

    	if(xTaskGetTickCount() >  last_ping + 500){
    		TASK_CAN_connect(&can1,id, 1);
			last_ping = xTaskGetTickCount();
		}

    	if(c!=0){
    		if(c!=CTRL_C){
    			ctrl_cnt = 2;
    		}
			xStreamBufferSend(main_can.tx_stream, &c, 1, 100);
    	}

    	uint8_t buffer[64];
    	uint32_t len= xStreamBufferReceive(main_can.rx_stream, buffer, sizeof(buffer), 1);
    	if(len){
			xSemaphoreTake(port->term_block, portMAX_DELAY);

			ttprintf(NULL, buffer, len);

			xSemaphoreGive(port->term_block);
		}



        c=0;
        xStreamBufferReceive(handle->currProgram->inputStream,&c,sizeof(c),pdMS_TO_TICKS(0));
        if(c==CTRL_C) {
        	ctrl_cnt--;
        	if(ctrl_cnt==1){
        		ttprintf(ctrl_text);
        		vTaskDelay(500);
        		TERM_sendVT100Code(handle, _VT100_CURSOR_BACK_BY, sizeof(ctrl_text)-1);
        	}

        }

    }while(ctrl_cnt);

    while(TASK_CAN_connect(&can1,id, 0)==0){
		vTaskDelay(1);
	}

    xSemaphoreGive(main_can.tx_semaphore);
    xSemaphoreGive(main_can.term_block);
#endif
    TERM_sendVT100Code(handle,_VT100_CURSOR_ENABLE, 0);
    TERM_killProgramm(handle);
}

static uint8_t INPUT_handler(TERMINAL_HANDLE * handle, uint16_t c){
    if(handle->currProgram->inputStream==NULL) return TERM_CMD_EXIT_SUCCESS;
#if RAW_INPUT==1
    uint8_t s_c = c;
    xStreamBufferSend(handle->currProgram->inputStream,&s_c,1,20);
#else
  	xStreamBufferSend(handle->currProgram->inputStream,&c,2,20);
#endif
  	return TERM_CMD_CONTINUE;
}
