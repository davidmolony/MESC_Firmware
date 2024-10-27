/*
 * TTerm
 *
 * Copyright (c) 2020 Thorben Zethoff, Jens Kerrinnes
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

#include "app_encoder.h"
#include "string.h"
#include "AXIS/MESCinterface.h"



//**** Give the app a name, description and stack size ****
#define APP_NAME "calibrate"
#define APP_DESCRIPTION "Encoder calibration"
#define APP_STACK 400
//*********************************************************

static uint8_t CMD_main(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
static void TASK_main(void *pvParameters);
static uint8_t INPUT_handler(TERMINAL_HANDLE * handle, uint16_t c);

//Name the function as your app
uint8_t REGISTER_app_encoder(TermCommandDescriptor * desc){
    TERM_addCommand(CMD_main, APP_NAME, APP_DESCRIPTION, 0, desc);
    return pdTRUE;
}

// Do not touch area ***************************************
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
//*********************************************************

static void bargraph(TERMINAL_HANDLE * handle, float min, float max, float val){
	char buffer[45];
	memset(buffer,0,45);
	buffer[0]='|';

	float norm = 40.0/max*val;

	for(int i=0; i<40;i++){
		buffer[i+1] = i<norm? '#' : '_';
	}
	buffer[41]='|';
	ttprintf("%s %10.2f ", buffer, (double)val);

}


#define ARROW_UP 	0x1005
#define ARROW_DOWN 	0x1006
#define ARROW_LEFT 	0x1004
#define ARROW_RIGHT 0x1003

#define MAX_ITEMS	3

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
    TERMINAL_HANDLE * handle = (TERMINAL_HANDLE*)pvParameters;  //Copy the terminal handle pointer
    uint16_t c=0;	//16 bit char, because it makes decoding of special 2 bytes command easier (arrow up/down for example)

    int selected=0;

    TERM_setCursorPos(handle, MAX_ITEMS*3+14, 0);
    ttprintf("Keys: [UP/DOWN] Select item  [z] Zero [s] Span [o] Offset [q] Quit");

    //*** App loop **************************************************************************************
    do{


    	TERM_sendVT100Code(handle,_VT100_CURSOR_DISABLE, 0);


    	TERM_setCursorPos(handle, 1, 0);
    	highlight(handle, "Throttle calibrated:", 0, selected);
    	TERM_setCursorPos(handle, 2, 0);
    	bargraph(handle, 0.0f, 1.0f, axis_vars.throttle_calibrated);


		if(c=='z'){
			axis_vars.throttle_start = axis_vars.throttle_raw;
		}
		if(c=='s'){
			axis_vars.throttle_end = axis_vars.throttle_z_calib;
		}

		if(c=='+'){
			axis_vars.throttle_raw += 0.1f;
			if(axis_vars.throttle_raw > 1.0f){
				axis_vars.throttle_raw = 0.0f;
			}
		}
		if(c=='-'){
			axis_vars.throttle_raw -= 0.1f;
			if(axis_vars.throttle_raw < 0.0f){
				axis_vars.throttle_raw = 1.0f;
			}
		}


    	TERM_setCursorPos(handle, 3, 0);
    	highlight(handle, "Throttle command to ESC:", 1, selected);
    	TERM_setCursorPos(handle, 4, 0);
    	bargraph(handle, -1.0f, 1.0f, axis_vars.throttle_mapped);

    	if(c=='o'){
			axis_vars.throttle_offset = axis_vars.throttle_calibrated;
		}

    	TERM_setCursorPos(handle, 5, 0);
    	ttprintf("Zero: %1.2f Span: %1.2f Offset: %1.2f", axis_vars.throttle_start, axis_vars.throttle_end, axis_vars.throttle_offset);


    	TERM_setCursorPos(handle, 8, 0);
    	highlight(handle, "Encoder SPI:", 2, selected);
    	TERM_setCursorPos(handle, 9, 0);
    	bargraph(handle, 0.0f, 1.0f, axis_vars.ratioSPI);

    	TERM_setCursorPos(handle, 10, 0);
    	highlight(handle, "Encoder PWM:", 3, selected);
    	TERM_setCursorPos(handle, 11, 0);
    	bargraph(handle, 0.0f, 1.0f, axis_vars.ratioPWM);






    	TERM_setCursorPos(handle, 12, 0);
    	ttprintf("RAW: %6d ERROR: %d CODE: %d", axis_vars.mt6816.angle, axis_vars.mt6816.error, axis_vars.mt6816.status);
    	TERM_setCursorPos(handle, 13, 0);
    	ttprintf("ERROR COUNT: %6d ACCUMULATED ERRORS: %d", axis_vars.error_count, axis_vars.accumulated_errors);

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
    }while(c!=CTRL_C && c!='q');
    //***************************************************************************************************
    TERM_sendVT100Code(handle,_VT100_CURSOR_ENABLE, 0);		//Enable cursor
    TERM_killProgramm(handle);	//Kill program and free memory
}

static uint8_t INPUT_handler(TERMINAL_HANDLE * handle, uint16_t c){
    if(handle->currProgram->inputStream==NULL) return TERM_CMD_EXIT_SUCCESS;	//Only do when input stream ptr is not NULL
  	xStreamBufferSend(handle->currProgram->inputStream,&c,2,20);	//Copy a char (16 bit) to the input buffer
  	return TERM_CMD_CONTINUE;
}
