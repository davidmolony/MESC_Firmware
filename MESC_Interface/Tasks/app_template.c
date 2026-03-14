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

#include "top.h"
#include "string.h"


//**** Give the app a name, description and stack size ****
#define APP_NAME "template"
#define APP_DESCRIPTION "hello world"
#define APP_STACK 400
//*********************************************************

static uint8_t CMD_main(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
static void TASK_main(void *pvParameters);
static uint8_t INPUT_handler(TERMINAL_HANDLE * handle, uint16_t c);

//Name the function as your app
uint8_t REGISTER_app_template(TermCommandDescriptor * desc){
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

static void TASK_main(void *pvParameters){
    TERMINAL_HANDLE * handle = (TERMINAL_HANDLE*)pvParameters;  //Copy the terminal handle pointer
    uint16_t c=0;	//16 bit char, because it makes decoding of special 2 bytes command easier (arrow up/down for example)

    //*** App loop **************************************************************************************
    do{

        ttprintf("Hello World\r\n");

    	c=0;
        xStreamBufferReceive(handle->currProgram->inputStream,&c,sizeof(c),pdMS_TO_TICKS(1000));  //Receive one char (16 bit) out of the user input with a 1000 tick timeout
    }while(c!=CTRL_C);
    //***************************************************************************************************
    TERM_sendVT100Code(handle,_VT100_CURSOR_ENABLE, 0);		//Enable cursor
    TERM_killProgramm(handle);	//Kill program and free memory
}

static uint8_t INPUT_handler(TERMINAL_HANDLE * handle, uint16_t c){
    if(handle->currProgram->inputStream==NULL) return TERM_CMD_EXIT_SUCCESS;	//Only do when input stream ptr is not NULL
  	xStreamBufferSend(handle->currProgram->inputStream,&c,2,20);	//Copy a char (16 bit) to the input buffer
  	return TERM_CMD_CONTINUE;
}
