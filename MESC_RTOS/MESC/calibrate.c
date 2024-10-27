/*
 **
 ******************************************************************************
 * @file           : calibrate.c
 * @brief          : ADC input calibration app
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


#include <MESC/calibrate.h>
#include "string.h"
#include "MESCfoc.h"

#define APP_NAME "calibrate"
#define APP_DESCRIPTION "Calibrates inputs"
#define APP_STACK 512

static uint8_t CMD_main(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
static void TASK_main(void *pvParameters);
static uint8_t INPUT_handler(TERMINAL_HANDLE * handle, uint16_t c);


uint8_t REGISTER_calibrate(TermCommandDescriptor * desc){
    TERM_addCommand(CMD_main, APP_NAME, APP_DESCRIPTION, 0, desc); 
    return pdTRUE;
}

extern float remote_adc[2];

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

static void bargraph(TERMINAL_HANDLE * handle, float min, float max, float val){
	char buffer[45];
	memset(buffer,0,45);
	buffer[0]='|';

	float norm = 40.0/max*val;

	for(int i=0; i<40;i++){
		buffer[i+1] = i<norm? '#' : '_';
	}
	buffer[41]='|';
	ttprintf("%s %10.0f ", buffer, (double)val);

}


#define ARROW_UP 	0x1005
#define ARROW_DOWN 	0x1006
#define ARROW_LEFT 	0x1004
#define ARROW_RIGHT 0x1003

#define MAX_ITEMS	4

static void highlight(TERMINAL_HANDLE * handle, char * text ,int index, int count){
	if(count==index){
		TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_GREEN);
	}else{
		TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
	}
	ttprintf("%s", text);
	TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
}


static void TASK_main(void *pvParameters){

	MESC_motor_typedef * motor_curr = &mtr[0];

    TERMINAL_HANDLE * handle = (TERMINAL_HANDLE*)pvParameters;
    TERM_sendVT100Code(handle,_VT100_CURSOR_DISABLE, 0);
    uint16_t c=0;
    int selected=0;

    TERM_setCursorPos(handle, MAX_ITEMS*3+3, 0);
    ttprintf("Keys: [UP/DOWN] Select item, [-/+] Save min/max value, [r] Reset MIN/MAX, [i] change polarity [CTRL+C] Quit");

    do{
    	TERM_sendVT100Code(handle,_VT100_CURSOR_DISABLE, 0);
    	TERM_setCursorPos(handle, 1, 0);
    	highlight(handle, "ADC1:", 0, selected);
    	TERM_setCursorPos(handle, 2, 0);
    	bargraph(handle, 0, 4095, motor_curr->Raw.ADC_in_ext1);
    	ttprintf("MIN: %6d MAX: %6d INV: %1.0f", (double)motor_curr->input_vars.adc1_MIN, (double)motor_curr->input_vars.adc1_MAX, (double)motor_curr->input_vars.ADC1_polarity);
    	TERM_setCursorPos(handle, 3, 0);
    	bargraph(handle, 0.0f, motor_curr->input_vars.max_request_Idq.q, motor_curr->input_vars.ADC1_req);
    	ttprintf("Request: %f", (double)motor_curr->input_vars.ADC1_req);

    	if(selected==0){
			if(c=='r'){
				motor_curr->input_vars.adc1_MIN=0;
				motor_curr->input_vars.adc1_MAX=4095;
			}
			if(c=='-'){
				motor_curr->input_vars.adc1_MIN=motor_curr->Raw.ADC_in_ext1;
			}
			if(c=='+'){
				motor_curr->input_vars.adc1_MAX=motor_curr->Raw.ADC_in_ext1;
			}
			if(c=='i'){
				motor_curr->input_vars.ADC1_polarity *= -1.0f;
			}
    	}


    	TERM_setCursorPos(handle, 4, 0);
    	highlight(handle, "ADC2:", 1, selected);
		TERM_setCursorPos(handle, 5, 0);
		bargraph(handle, 0, 4095, 0);
		ttprintf("MIN: %6d MAX: %6d INV: %1.0f", (double)motor_curr->input_vars.adc2_MIN, (double)motor_curr->input_vars.adc2_MAX, (double)motor_curr->input_vars.ADC2_polarity);
    	TERM_setCursorPos(handle, 6, 0);
    	bargraph(handle, 0.0f, motor_curr->input_vars.max_request_Idq.q, motor_curr->input_vars.ADC1_req);
    	ttprintf("Request: %f", (double)motor_curr->input_vars.ADC1_req);

		if(selected==1){
			if(c=='r'){
				motor_curr->input_vars.adc2_MIN=0;
				motor_curr->input_vars.adc2_MAX=4095;
			}
			if(c=='-'){
				motor_curr->input_vars.adc2_MIN=100;
			}
			if(c=='+'){
				motor_curr->input_vars.adc2_MAX=100;
			}
			if(c=='i'){
				motor_curr->input_vars.ADC2_polarity *= -1.0f;
			}
		}


    	TERM_setCursorPos(handle, 7, 0);
    	highlight(handle, "Remote ADC1:", 2, selected);
		TERM_setCursorPos(handle, 8, 0);
    	bargraph(handle, 0.0f, 1.0f, motor_curr->input_vars.remote_ADC1_req);
    	ttprintf("Request: %f", (double)motor_curr->input_vars.remote_ADC1_req);

    	TERM_setCursorPos(handle, 9, 0);
    	highlight(handle, "Remote ADC2:", 2, selected);
		TERM_setCursorPos(handle, 10, 0);
    	bargraph(handle, 0.0f, 1.0f, motor_curr->input_vars.remote_ADC2_req);
    	ttprintf("Request: %f", (double)motor_curr->input_vars.remote_ADC2_req);

		if(c==ARROW_DOWN){
			selected++;
			if(selected>MAX_ITEMS) selected = 0;
		}
		if(c==ARROW_UP){
			selected--;
			if(selected<0) selected = MAX_ITEMS-1;
		}


        c=0;
        xStreamBufferReceive(handle->currProgram->inputStream,&c,sizeof(c),pdMS_TO_TICKS(100));
    }while(c!=CTRL_C);
    TERM_sendVT100Code(handle,_VT100_CURSOR_ENABLE, 0);
    TERM_killProgramm(handle);
}

static uint8_t INPUT_handler(TERMINAL_HANDLE * handle, uint16_t c){
    if(handle->currProgram->inputStream==NULL) return TERM_CMD_EXIT_SUCCESS;
  	xStreamBufferSend(handle->currProgram->inputStream,&c,2,20);
  	return TERM_CMD_CONTINUE;
}
