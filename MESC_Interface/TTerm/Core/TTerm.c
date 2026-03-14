/*
 * TTerm
 *
 * Copyright (c) 2020 Thorben Zethoff
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
    
#if PIC32 == 1
#include <xc.h>
#endif    
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

//#include "apps.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
//#include "UART.h"
#include "TTerm/Core/include/TTerm.h"
#include "TTerm/Core/include/TTerm_cmd.h"
#include "TTerm/Core/include/TTerm_AC.h"
#include "TTerm/Core/include/TTerm_cwd.h"
#include <TTerm/Core/include/TTerm_var.h>

TermCommandDescriptor TERM_defaultList = {.nextCmd = 0, .commandLength = 0};

#if TERM_SUPPORT_VARIABLES
TermVariableDescriptor TERM_varList = {.nextVar = 0, .nameLength = 0};
#endif

uint8_t TERM_baseCMDsAdded = 0;


#define ESC_STR "\x1b"

#if EXTENDED_PRINTF == 1
TERMINAL_HANDLE * TERM_createNewHandle(TermPrintHandler printFunction, void * port, unsigned echoEnabled, TermCommandDescriptor * cmdListHead, TermErrorPrinter errorPrinter, const char * usr){
#else
TERMINAL_HANDLE * TERM_createNewHandle(TermPrintHandler printFunction, unsigned echoEnabled, TermCommandDescriptor * cmdListHead, TermErrorPrinter errorPrinter, const char * usr){    
#endif    
    
    //reserve memory
    TERMINAL_HANDLE * newHandle = pvPortMalloc(sizeof(TERMINAL_HANDLE));
    memset(newHandle, 0, sizeof(TERMINAL_HANDLE));
    
    newHandle->inputBuffer = pvPortMalloc(TERM_INPUTBUFFER_SIZE);
    newHandle->currUserName = pvPortMalloc(strlen(usr) + 1 + strlen(TERM_getVT100Code(_VT100_FOREGROUND_COLOR, _VT100_YELLOW)) + strlen(TERM_getVT100Code(_VT100_RESET_ATTRIB, 0)));
    newHandle->userPermissionLevel = 0;
    newHandle->currPermissionLevel = newHandle->userPermissionLevel;
    
    //initialise function pointers
    newHandle->print = printFunction;  
    
    if(errorPrinter == 0){
        newHandle->errorPrinter = TERM_defaultErrorPrinter;
    }else{
        newHandle->errorPrinter = errorPrinter;
    }
    
    //initialise data
    #if EXTENDED_PRINTF == 1
    newHandle->port = port;
    #endif  
    
    newHandle->echoEnabled = echoEnabled;
    newHandle->cmdListHead = cmdListHead;

    #if TERM_SUPPORT_VARIABLES
    newHandle->varHandle = pvPortMalloc(sizeof(TermVariableHandle));
    newHandle->varHandle->varListHead = &TERM_varList;
	#endif

    sprintf(newHandle->currUserName, "%s%s%s", TERM_getVT100Code(_VT100_FOREGROUND_COLOR, _VT100_BLUE), usr, TERM_getVT100Code(_VT100_RESET_ATTRIB, 0));
    
    //reset pointers
    newHandle->currEscSeqPos = 0xff;
    
    #if TERM_SUPPORT_CWD == 1
    newHandle->cwdPath = pvPortMalloc(2);
    strcpy(newHandle->cwdPath, "/");
    #endif

    //if this is the first console we initialize we need to add the static commands
    if(!TERM_baseCMDsAdded){
        TERM_baseCMDsAdded = 1;
        
        TERM_addCommand(CMD_help, "help", "Displays this help message", 0, &TERM_defaultList);
        TERM_addCommand(CMD_cls, "cls", "Clears the screen", 0, &TERM_defaultList);
        TERM_addCommand(CMD_reset, "reset", "resets the device", 0, &TERM_defaultList);
        
        #if TERM_SUPPORT_CWD == 1
        TERM_addCommand(CMD_ls, "ls", "List directory", 0, &TERM_defaultList);
        TERM_addCommand(CMD_cat, "cat", "^^ mew", 0, &TERM_defaultList);
        TERM_addCommand(CMD_echo, "echo", "echo echo", 0, &TERM_defaultList);
        TERM_addCommand(CMD_cd, "cd", "Change directory", 0, &TERM_defaultList);
        TERM_addCommand(CMD_mkdir, "mkdir", "Make directory", 0, &TERM_defaultList);
        TERM_addCommand(CMD_rm, "rm", "Remove file/directory", 0, &TERM_defaultList);
        #endif  
        
		#if TERM_SUPPORT_VARIABLES
        TermCommandDescriptor * varAC = TERM_addCommand(CMD_varList, "get", "Get variable value", 0, &TERM_defaultList);
        TERM_addCommandAC(varAC, TERM_varCompleter, newHandle->varHandle->varListHead);
        varAC = TERM_addCommand(CMD_varSet, "set", "Set variable", 0, &TERM_defaultList);
        TERM_addCommandAC(varAC, TERM_varCompleter, newHandle->varHandle->varListHead);

        TERM_addCommand(CMD_varSave, "save", "Save variables", 0, &TERM_defaultList);

        varAC = TERM_addCommand(CMD_varLoad, "load", "Load variables", 0, &TERM_defaultList);
        TERM_addCommandAC(varAC, TERM_varCompleter, newHandle->varHandle->varListHead);

        TERM_addCommand(CMD_su, "su", "Become superuser", 0, &TERM_defaultList);

        varAC = TERM_addCommand(CMD_varChown, "chown", "Change owner", 0, &TERM_defaultList);
        TERM_addCommandAC(varAC, TERM_varCompleter, newHandle->varHandle->varListHead);

        CMD_varLoad(newHandle, 0, NULL);

		#endif

      
        TermCommandDescriptor * test = TERM_addCommand(CMD_testCommandHandler, "test", "tests stuff", 0, &TERM_defaultList);
        head = ACL_create();
        ACL_add(head, "-ra");
        ACL_add(head, "-r");
        ACL_add(head, "-aa");
        TERM_addCommandAC(test, ACL_defaultCompleter, head);
        
       // REGISTER_apps(&TERM_defaultList);
    }

#ifdef TERM_ENABLE_STARTUP_TEXT
    //TODO VT100 reset at boot
    //TODO add min start frame to signal that debugging started and print this again
    //TODO colors in the boot message
    TERM_printBootMessage(newHandle);
#endif
    return newHandle;
}

void TERM_destroyHandle(TERMINAL_HANDLE * handle){
    if(handle->currProgram != NULL){
        //try to gracefully shutdown the running program
        (*handle->currProgram->inputHandler)(handle, 0x03);
        vTaskDelay(10);
    }
    
    vPortFree(handle->inputBuffer);
    vPortFree(handle->currUserName);
    
#if TERM_SUPPORT_VARIABLES
    vPortFree(handle->varHandle);
#endif


    uint8_t currHistoryPos = 0;
    for(;currHistoryPos < TERM_HISTORYSIZE; currHistoryPos++){
        if(handle->historyBuffer[currHistoryPos] != 0){
            vPortFree(handle->historyBuffer[currHistoryPos]);
            handle->historyBuffer[currHistoryPos] = 0;
        }
    }
    
    vPortFree(handle->autocompleteBuffer);
    handle->autocompleteBuffer = NULL;
    vPortFree(handle);
}

void TERM_printDebug(TERMINAL_HANDLE * handle, char * format, ...){
    //TODO make this nicer... we don't need a double buffer allocation, we should instead send the va_list to the print function. But it is way to late at night for me to code this now...
    //TODO implement a debug level control in the terminal handle (permission level?)
    va_list arg;
    va_start(arg, format);
    
    char * buff = (char*) pvPortMalloc(256);
    vsnprintf(buff, 256,format, arg);
    
    ttprintfEcho("\r\n%s", buff);
    
    if(handle->currBufferLength == 0){
        ttprintfEcho("%s@%s>", handle->currUserName, TERM_DEVICE_NAME);
    }else{
        ttprintfEcho("%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->inputBuffer);
        if(handle->inputBuffer[handle->currBufferPosition] != 0) TERM_sendVT100Code(handle, _VT100_CURSOR_BACK_BY, handle->currBufferLength - handle->currBufferPosition);
    }
    
    vPortFree(buff);
    
    va_end(arg);
}

uint8_t TERM_processBuffer(uint8_t * data, uint16_t length, TERMINAL_HANDLE * handle){
    uint16_t currPos = 0;
    for(;currPos < length; currPos++){

    	if(handle->currProgram != NULL && handle->currProgram->raw_input == 1){
    		TERM_handleInput(data[currPos], handle);
    	}else{
    	//ttprintfEcho("checking 0x%02x\r\n", data[currPos]);
			if(handle->currEscSeqPos != 0xff){
				if(handle->currEscSeqPos == 0){
					if(data[currPos] == '['){
						if(handle->currEscSeqPos < TTERM_ESC_SEQ_BUFFER_SIZE){
							handle->escSeqBuff[handle->currEscSeqPos] = data[currPos];
							handle->currEscSeqPos++;
						}
					}else{
						switch(data[currPos]){
							case 'c':
								handle->currEscSeqPos = 0xff;
								TERM_handleInput(_VT100_RESET, handle);
								break;
							case 0x1b:
								handle->currEscSeqPos = 0;
								break;
							default:
								handle->currEscSeqPos = 0xff;
								TERM_handleInput(0x1b, handle);
								TERM_handleInput(data[currPos], handle);
								break;
						}
					}
				}else{
					if(isACIILetter(data[currPos])){
						if(data[currPos] == 'n'){
							if(handle->currEscSeqPos == 2){
								if(handle->escSeqBuff[0] == '5'){        //Query device status
								}else if(handle->escSeqBuff[0] == '6'){  //Query cursor position
								}
							}
						}else if(data[currPos] == 'c'){
							if(handle->currEscSeqPos == 1){              //Query device code
							}
						}else if(data[currPos] == 'F'){
							if(handle->currEscSeqPos == 1){              //end
								TERM_handleInput(_VT100_KEY_END, handle);
							}
						}else if(data[currPos] == 'H'){
							if(handle->currEscSeqPos == 1){              //pos1
								TERM_handleInput(_VT100_KEY_POS1, handle);
							}
						}else if(data[currPos] == 'C'){                      //cursor forward
							if(handle->currEscSeqPos > 1){
								handle->escSeqBuff[handle->currEscSeqPos] = 0;
							}else{
								TERM_handleInput(_VT100_CURSOR_FORWARD, handle);
							}
						}else if(data[currPos] == 'D'){                      //cursor backward
							if(handle->currEscSeqPos > 1){
								handle->escSeqBuff[handle->currEscSeqPos] = 0;
							}else{
								TERM_handleInput(_VT100_CURSOR_BACK, handle);
							}
						}else if(data[currPos] == 'A'){                      //cursor up
							if(handle->currEscSeqPos > 1){
								handle->escSeqBuff[handle->currEscSeqPos] = 0;
							}else{
								TERM_handleInput(_VT100_CURSOR_UP, handle);
							}
						}else if(data[currPos] == 'B'){                      //cursor down
							if(handle->currEscSeqPos > 1){
								handle->escSeqBuff[handle->currEscSeqPos] = 0;
							}else{
								TERM_handleInput(_VT100_CURSOR_DOWN, handle);
							}
						}else if(data[currPos] == 'Z'){                      //shift tab or ident request(at least from exp.; didn't find any official spec containing this)
							TERM_handleInput(_VT100_BACKWARDS_TAB, handle);

						}else if(data[currPos] == '~'){                      //Tilde the end of a special key command (Esc[{keyID}~)
							if(handle->escSeqBuff[1] == 0x32){            //insert
								TERM_handleInput(_VT100_KEY_INS, handle);
							}else if(handle->escSeqBuff[1] == 0x33){      //delete
								TERM_handleInput(_VT100_KEY_DEL, handle);
							}else if(handle->escSeqBuff[1] == 0x35){      //page up
								TERM_handleInput(_VT100_KEY_PAGE_UP, handle);
							}else if(handle->escSeqBuff[1] == 0x36){      //page down
								TERM_handleInput(_VT100_KEY_PAGE_DOWN, handle);
							}else{
								TERM_handleInput(_VT100_INVALID, handle);
							}
						}else{                      //others
							handle->escSeqBuff[handle->currEscSeqPos+1] = 0;
						}
						handle->currEscSeqPos = 0xff;
					}else{
						handle->escSeqBuff[handle->currEscSeqPos++] = data[currPos];
					}
				}
			}else{
				if(data[currPos] == 0x1B){     //ESC for V100 control sequences
					handle->currEscSeqPos = 0;
				}else{
					TERM_handleInput(data[currPos], handle);
				}
			}
    	}
    }
    return length;
}

unsigned isACIILetter(char c){
    return (c > 64 && c < 91) || (c > 96 && c < 122) || c == '~';
}

void TERM_printBootMessage(TERMINAL_HANDLE * handle){
    TERM_sendVT100Code(handle, _VT100_RESET, 0); TERM_sendVT100Code(handle, _VT100_CURSOR_POS1, 0); TERM_sendVT100Code(handle, _VT100_WRAP_OFF, 0);
    ttprintfEcho("\r\n%s\r\n", TERM_startupText1);
    ttprintfEcho("%s\r\n", TERM_startupText2);
    ttprintfEcho("%s\r\n", TERM_startupText3);
    
    if(handle->currBufferLength == 0){
        ttprintfEcho("\r\n\r\n%s@%s>", handle->currUserName, TERM_DEVICE_NAME);
    }else{
        ttprintfEcho("\r\n\r\n%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->inputBuffer);
        if(handle->inputBuffer[handle->currBufferPosition] != 0) TERM_sendVT100Code(handle, _VT100_CURSOR_BACK_BY, handle->currBufferLength - handle->currBufferPosition);
    }
}

BaseType_t ptr_is_in_ram(void* ptr){
    if(ptr > (void*)START_OF_FLASH && ptr < (void*)END_OF_FLASH){
        return pdFALSE;
    }
    return pdTRUE;
}

uint8_t TERM_defaultErrorPrinter(TERMINAL_HANDLE * handle, uint32_t retCode){
    switch(retCode){
        case TERM_CMD_EXIT_SUCCESS:
            ttprintfEcho("\r\n%s@%s>", handle->currUserName, TERM_DEVICE_NAME);
            break;

        case TERM_CMD_EXIT_ERROR:
            ttprintfEcho("\r\nTask returned with error code %d\r\n%s@%s>", retCode, handle->currUserName, TERM_DEVICE_NAME);
            break;

        case TERM_CMD_EXIT_NOT_FOUND:
        	ttprintfEcho("\"%s\"", handle->inputBuffer);
            ttprintfEcho(" is not a valid command. Type \"help\" to see a list of available ones\r\n%s@%s>", handle->currUserName, TERM_DEVICE_NAME);
            break;
    }
    return 0;
}

uint8_t TERM_handleInput(uint16_t c, TERMINAL_HANDLE * handle){
    //ttprintfEcho("received 0x%04x\r\n", c);
    if(handle->currProgram != NULL){
        //call the handler of the current override
        uint8_t currRetCode = (*handle->currProgram->inputHandler)(handle, c);
        
        (*handle->errorPrinter)(handle, currRetCode);
        
        return 1;
    }
    
    switch(c){
        case '\r':      //enter
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);
            
            if(handle->currBufferLength != 0){
                ttprintfEcho("\r\n", handle->inputBuffer);

                if(handle->historyBuffer[handle->currHistoryWritePosition] != 0){
                    vPortFree(handle->historyBuffer[handle->currHistoryWritePosition]);
                    handle->historyBuffer[handle->currHistoryWritePosition] = 0;
                }

                handle->historyBuffer[handle->currHistoryWritePosition] = pvPortMalloc(handle->currBufferLength + 1);
                memcpy(handle->historyBuffer[handle->currHistoryWritePosition], handle->inputBuffer, handle->currBufferLength + 1);
                
                if(++handle->currHistoryWritePosition >= TERM_HISTORYSIZE) handle->currHistoryWritePosition = 0;
                
                handle->currHistoryReadPosition = handle->currHistoryWritePosition;

                uint8_t retCode = TERM_interpretCMD(handle->inputBuffer, handle->currBufferLength, handle);
                (*handle->errorPrinter)(handle, retCode);
                
                handle->currBufferPosition = 0;
                handle->currBufferLength = 0;
                handle->inputBuffer[handle->currBufferPosition] = 0;
            }else{
                ttprintfEcho("\r\n%s@%s>", handle->currUserName, TERM_DEVICE_NAME);
            }       
            break;
            
        case 0x03:      //CTRL+c
        	handle->currBufferPosition=0;
        	handle->currBufferLength=0;
        	handle->inputBuffer[0]=0;
        	TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
        	ttprintfEcho("\r%s@%s>", handle->currUserName, TERM_DEVICE_NAME);
            break;
        case 0x0a: 		//LF
        case 0x08:      //backspace (used by xTerm)
        case 0x7f:      //DEL       (used by hTerm)
        case _VT100_KEY_DEL:
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);

            if(handle->currBufferPosition < handle->currBufferLength && c == _VT100_KEY_DEL){
                handle->currBufferPosition ++;
                TERM_sendVT100Code(handle, _VT100_CURSOR_FORWARD, 0);
            }else if (c == _VT100_KEY_DEL) {
                break;
            }

            if(handle->currBufferPosition == 0) break;

            if(handle->inputBuffer[handle->currBufferPosition] != 0){      //check if we are at the end of our command
                //we are somewhere in the middle -> move back existing characters
                strsft(handle->inputBuffer, handle->currBufferPosition - 1, -1);
                ttprintfEcho("\x08");   
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE_END, 0);
                ttprintfEcho("%s", &handle->inputBuffer[handle->currBufferPosition - 1]);
                TERM_sendVT100Code(handle, _VT100_CURSOR_BACK_BY, handle->currBufferLength - handle->currBufferPosition);
                handle->currBufferPosition --;
                handle->currBufferLength --;
            }else{
                //we are somewhere at the end -> just delete the current one
                handle->inputBuffer[--handle->currBufferPosition] = 0;
                ttprintfEcho("\x08 \x08");           
                handle->currBufferLength --;
            }
            break;
            
        case _VT100_KEY_END:
        	TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST); //is the user browsing the history right now? if so copy whatever ehs looking at to the entry buffer

			if(handle->currBufferLength > handle->currBufferPosition){
				//move the cursor to the right position
				TERM_sendVT100Code(handle, _VT100_CURSOR_FORWARD_BY, handle->currBufferLength - handle->currBufferPosition);
				handle->currBufferPosition = handle->currBufferLength;
			}
            break;
            
        case _VT100_KEY_POS1:
        	TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST); //is the user browsing the history right now? if so copy whatever ehs looking at to the entry buffer

			if(handle->currBufferPosition > 0){
				//move the cursor to the right position
				TERM_sendVT100Code(handle, _VT100_CURSOR_BACK_BY, handle->currBufferPosition);
				handle->currBufferPosition = 0;
			}
            break;
            
        case _VT100_CURSOR_FORWARD:
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);
            
            if(handle->currBufferPosition < handle->currBufferLength){
                handle->currBufferPosition ++;
                TERM_sendVT100Code(handle, _VT100_CURSOR_FORWARD, 0);
            }
            break;
            
        case _VT100_CURSOR_BACK:
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);
            
            if(handle->currBufferPosition > 0){
                handle->currBufferPosition --;
                TERM_sendVT100Code(handle, _VT100_CURSOR_BACK, 0);
            }
            break;
            
        case _VT100_CURSOR_UP:
            TERM_checkForCopy(handle, TERM_CHECK_COMP);
            
            do{
                if(--handle->currHistoryReadPosition >= TERM_HISTORYSIZE) handle->currHistoryReadPosition = TERM_HISTORYSIZE - 1;
                
                if(handle->historyBuffer[handle->currHistoryReadPosition] != 0){
                    break;
                }
            }while(handle->currHistoryReadPosition != handle->currHistoryWritePosition);
            
            //print out the command at the current history read position
            if(handle->currHistoryReadPosition == handle->currHistoryWritePosition){
                ttprintfEcho("\x07");   //rings a bell doesn't it?                                                      
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                ttprintfEcho("\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->inputBuffer);
            }else{
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                ttprintfEcho("\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->historyBuffer[handle->currHistoryReadPosition]);
            }
            break;
            
        case _VT100_CURSOR_DOWN:
            TERM_checkForCopy(handle, TERM_CHECK_COMP);
            
            while(handle->currHistoryReadPosition != handle->currHistoryWritePosition){
                if(++handle->currHistoryReadPosition >= TERM_HISTORYSIZE) handle->currHistoryReadPosition = 0;
                
                if(handle->historyBuffer[handle->currHistoryReadPosition] != 0){
                    break;
                }
            }
            
            //print out the command at the current history read position
            if(handle->currHistoryReadPosition == handle->currHistoryWritePosition){
                ttprintfEcho("\x07");   //rings a bell doesn't it?                                                      
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                ttprintfEcho("\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->inputBuffer);
            }else{
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                ttprintfEcho("\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->historyBuffer[handle->currHistoryReadPosition]);
            }
            break;
            
        case '\t':      //tab
            TERM_checkForCopy(handle, TERM_CHECK_HIST);
            
            if(handle->autocompleteBuffer == NULL){ 
                TERM_doAutoComplete(handle);
            }
            
            if(++handle->currAutocompleteCount > handle->autocompleteBufferLength) handle->currAutocompleteCount = 0;
            
            if(handle->currAutocompleteCount == 0){
                ttprintfEcho("\x07");
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                ttprintfEcho("\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->inputBuffer);
            }else{
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                unsigned printQuotationMarks = strchr(handle->autocompleteBuffer[handle->currAutocompleteCount - 1], ' ') != 0;
                
                //workaround for inconsistent printf behaviour when using %.*s with length 0
                if(handle->autocompleteStart == 0){
                    ttprintfEcho(printQuotationMarks ? "\r%s@%s>\"%s\"" : "\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->autocompleteBuffer[handle->currAutocompleteCount - 1]);
                }else{
                    ttprintfEcho(printQuotationMarks ? "\r%s@%s>%.*s\"%s\"" : "\r%s@%s>%.*s%s", handle->currUserName, TERM_DEVICE_NAME, handle->autocompleteStart, handle->inputBuffer, handle->autocompleteBuffer[handle->currAutocompleteCount - 1]);
                }
            }
            break;
            
        case _VT100_BACKWARDS_TAB:
            TERM_checkForCopy(handle, TERM_CHECK_HIST);
            
            if(handle->autocompleteBuffer == NULL){ 
                TERM_doAutoComplete(handle);
            }
            
            if(--handle->currAutocompleteCount > handle->autocompleteBufferLength - 1) handle->currAutocompleteCount = handle->autocompleteBufferLength - 1;
            
            if(handle->currAutocompleteCount == 0){
                ttprintfEcho("\x07");
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                ttprintfEcho("\r%s@%s>%s", handle->currUserName, TERM_DEVICE_NAME, handle->inputBuffer);
            }else{
                TERM_sendVT100Code(handle, _VT100_ERASE_LINE, 0);
                unsigned printQuotationMarks = strchr(handle->autocompleteBuffer[handle->currAutocompleteCount - 1], ' ') != 0;
                ttprintfEcho(printQuotationMarks ? "\r%s@%s>%.*s\"%s\"" : "\r%s@%s>%.*s%s", handle->currUserName, TERM_DEVICE_NAME, handle->autocompleteStart, handle->inputBuffer, handle->autocompleteBuffer[handle->currAutocompleteCount - 1]);
            }
            break;
            
        case _VT100_RESET:
            TERM_printBootMessage(handle);
            break;
           
        case 32 ... 126:
            TERM_checkForCopy(handle, TERM_CHECK_COMP_AND_HIST);

			//TODO check for string length overflow

			if(handle->inputBuffer[handle->currBufferPosition] != 0 && handle->currBufferLength < TERM_INPUTBUFFER_SIZE -2){      //check if we are at the end of our command
				strsft(handle->inputBuffer, handle->currBufferPosition, 1);
				handle->inputBuffer[handle->currBufferPosition] = c;
				TERM_sendVT100Code(handle, _VT100_ERASE_LINE_END, 0);
				ttprintfEcho("%s", &handle->inputBuffer[handle->currBufferPosition]);
				TERM_sendVT100Code(handle, _VT100_CURSOR_BACK_BY, handle->currBufferLength - handle->currBufferPosition);
				handle->currBufferLength ++;
				handle->currBufferPosition ++;
			}else{

				//we are at the end -> just delete the current one
				handle->inputBuffer[handle->currBufferPosition++] = c;
				handle->inputBuffer[handle->currBufferPosition] = 0;
				handle->currBufferLength ++;
				if(handle->currBufferPosition < TERM_INPUTBUFFER_SIZE -1){
					ttprintfEcho("%c", c);
				}
			}


			if(handle->currBufferPosition > TERM_INPUTBUFFER_SIZE -2){
				handle->currBufferPosition = TERM_INPUTBUFFER_SIZE -2;
			}
			if(handle->currBufferLength > TERM_INPUTBUFFER_SIZE -2){
				handle->currBufferLength = TERM_INPUTBUFFER_SIZE -2;
			}

            break;
            
        default:
            TERM_printDebug(handle, "unknown code received: 0x%02x\r\n", c);
            break;
    }
    return 1;
}

void TERM_checkForCopy(TERMINAL_HANDLE * handle, COPYCHECK_MODE mode){
    if((mode & TERM_CHECK_COMP) && handle->autocompleteBuffer != NULL){ 
        if(handle->currAutocompleteCount != 0){
            char * dst = (char *) ((uint32_t) handle->inputBuffer + handle->autocompleteStart);
            if(strchr(handle->autocompleteBuffer[handle->currAutocompleteCount - 1], ' ') != 0){
                sprintf(dst, "\"%s\"", handle->autocompleteBuffer[handle->currAutocompleteCount - 1]);
            }else{
                strcpy(dst, handle->autocompleteBuffer[handle->currAutocompleteCount - 1]);
            }
            handle->currBufferLength = strlen(handle->inputBuffer);
            handle->currBufferPosition = handle->currBufferLength;
            handle->inputBuffer[handle->currBufferPosition] = 0;
        }
        vPortFree(handle->autocompleteBuffer);
        handle->autocompleteBuffer = NULL;
    }
    
    if((mode & TERM_CHECK_HIST) && handle->currHistoryWritePosition != handle->currHistoryReadPosition){
        strcpy(handle->inputBuffer, handle->historyBuffer[handle->currHistoryReadPosition]);
        handle->currBufferLength = strlen(handle->inputBuffer);
        handle->currBufferPosition = handle->currBufferLength;
        handle->currHistoryReadPosition = handle->currHistoryWritePosition;
    }
}

char * strnchr(char * str, char c, uint32_t length){
    uint32_t currPos = 0;
    for(;currPos < length && str[currPos] != 0; currPos++){
        if(str[currPos] == c) return &str[currPos];
    }
    return NULL;
}

void strsft(char * src, int32_t startByte, int32_t offset){
    if(offset == 0) return;
    
    if(offset > 0){     //shift forward
        uint32_t currPos = strlen(src) + offset;
        src[currPos--] = 0;
        for(; currPos >= startByte; currPos--){
            if(currPos == 0){
                src[currPos] = ' ';
                break;
            }
            src[currPos] = src[currPos - offset];
        }
        return;
    }else{              //shift backward
        uint32_t currPos = startByte;
        for(; src[currPos - offset] != 0; currPos++){
            src[currPos] = src[currPos - offset];
        }
        src[currPos] = src[currPos - offset];
        return;
    }
}

TermCommandDescriptor * TERM_findCMD(TERMINAL_HANDLE * handle){
    uint8_t currPos = 0;
    uint16_t cmdLength = handle->currBufferLength;
    
    char * firstSpace = strchr(handle->inputBuffer, ' ');
    if(firstSpace != 0){
        cmdLength = (uint16_t) ((uint32_t) firstSpace - (uint32_t) handle->inputBuffer);
    }
    
    TermCommandDescriptor * currCmd = handle->cmdListHead->nextCmd;
    for(;currPos < handle->cmdListHead->commandLength; currPos++){
        if(currCmd->commandLength == cmdLength && strncmp(handle->inputBuffer, currCmd->command, cmdLength) == 0) return currCmd;
        currCmd = currCmd->nextCmd;
    }
    
    return NULL;
}

uint8_t TERM_interpretCMD(char * data, uint16_t dataLength, TERMINAL_HANDLE * handle){
    
    TermCommandDescriptor * cmd = TERM_findCMD(handle);
    
    if(cmd != 0){
        uint16_t argCount = TERM_countArgs(data, dataLength);
        if(argCount == TERM_ARGS_ERROR_STRING_LITERAL){
            ttprintfEcho("\r\nError: unclosed string literal in command\r\n");
            return TERM_CMD_EXIT_ERROR;
        }

        char ** args = 0;
        if(argCount != 0){
            args = pvPortMalloc(sizeof(char*) * argCount);
            TERM_seperateArgs(data, dataLength, args);
        }

        uint8_t retCode = TERM_CMD_EXIT_ERROR;
        if(cmd->function != 0){
            //configASSERT((cmd->function > 0x9d000000) && (cmd->function < 0x9d100000));
            retCode = (*cmd->function)(handle, argCount, args);
        }

        if(argCount != 0) vPortFree(args);
        return retCode;
    }
    
    return TERM_CMD_EXIT_NOT_FOUND;
}

uint8_t TERM_seperateArgs(char * data, uint16_t dataLength, char ** buff){
    uint8_t count = 0;
    uint8_t currPos = 0;
    unsigned quoteMark = 0;
    char * currStringStart = 0;
    char * lastSpace = 0;
    for(;currPos<dataLength; currPos++){
        switch(data[currPos]){
            case ' ':
                if(!quoteMark){
                    data[currPos] = 0;
                    lastSpace = &data[currPos + 1];
                }
                break;
                
            case '"':
                if(quoteMark){
                    quoteMark = 0;
                    if(currStringStart){
                        data[currPos] = 0;
                        buff[count++] = currStringStart;
                    }
                }else{
                    quoteMark = 1;
                    currStringStart = &data[currPos+1];
                }
                        
                break;
                
            default:
                if(!quoteMark){
                    if(lastSpace != 0){
                        buff[count++] = lastSpace;
                        lastSpace = 0;
                    }
                }
                break;
        }
    }
    //if(quoteMark) TERM_ARGS_ERROR_STRING_LITERAL;
    return count;
}

uint16_t TERM_countArgs(const char * data, uint16_t dataLength){
    uint8_t count = 0;
    uint8_t currPos = 0;
    unsigned quoteMark = 0;
    const char * currStringStart = NULL;
    const char * lastSpace = NULL;
    for(;currPos<dataLength; currPos++){
        switch(data[currPos]){
            case ' ':
                if(!quoteMark){
                    lastSpace = &data[currPos + 1];
                }
                break;
                
            case '"':
                if(quoteMark){
                    quoteMark = 0;
                    if(currStringStart){
                        count ++;
                    }
                }else{
                    quoteMark = 1;
                    currStringStart = &data[currPos+1];
                }
                        
                break;
            case 0:
                configASSERT(0);
                
            default:
                if(!quoteMark){
                    if(lastSpace){
                        count ++;
                        lastSpace = NULL;
                    }
                }
                break;
        }
    }
    if(quoteMark) return TERM_ARGS_ERROR_STRING_LITERAL;
    return count;
}

void TERM_freeCommandList(TermCommandDescriptor ** cl, uint16_t length){
    /*uint8_t currPos = 0;
    for(;currPos < length; currPos++){
        vPortFree(cl[currPos]);
    }*/
    vPortFree(cl);
}

TermCommandDescriptor * TERM_addCommand(TermCommandFunction function, const char * command, const char * description, uint8_t minPermissionLevel, TermCommandDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
        
    if(head->commandLength == 0xff) return 0;
    
    TermCommandDescriptor * newCMD = pvPortMalloc(sizeof(TermCommandDescriptor));
    
    newCMD->command = command;
    newCMD->commandDescription = description;
    newCMD->commandLength = strlen(command);
    newCMD->function = function;
    newCMD->minPermissionLevel = minPermissionLevel;
    newCMD->ACHandler = 0;
    
    TERM_LIST_add(newCMD, head);
    return newCMD;
}

void TERM_LIST_add(TermCommandDescriptor * item, TermCommandDescriptor * head){
    uint32_t currPos = 0;
    TermCommandDescriptor ** lastComp = &head->nextCmd;
    TermCommandDescriptor * currComp = head->nextCmd;
    
    while(currPos < head->commandLength){
        if(TERM_isSorted(currComp, item)){
            *lastComp = item;
            item->nextCmd = currComp;
            head->commandLength ++;
            
            return;
        }
        if(currComp->nextCmd == 0){
            item->nextCmd = currComp->nextCmd;
            currComp->nextCmd = item;
            head->commandLength ++;
            return;
        }
        lastComp = &currComp->nextCmd;
        currComp = currComp->nextCmd;
    }
    
    item->nextCmd = 0;
    *lastComp = item;
    head->commandLength ++;
}
/*
void ACL_remove(AC_LIST_HEAD * head, char * string){
    if(head->isConst || head->elementCount == 0) return;
    uint32_t currPos = 0;
    AC_LIST_ELEMENT ** lastComp = &head->first;
    AC_LIST_ELEMENT * currComp = head->first;
    
    for(;currPos < head->elementCount; currPos++){
        if((strlen(currComp->string) == strlen(string)) && (strcmp(currComp->string, string) == 0)){
            *lastComp = currComp->next;
            
            //TODO make this portable
            if(ptr_is_in_ram(currComp->string)){
                vPortFree(currComp->string);
            }
            
            vPortFree(currComp);
            head->elementCount --;
            return;
        }
        if(currComp->next == 0) break;
        lastComp = &currComp->next;
        currComp = currComp->next;
    }
}
*/

unsigned TERM_isSorted(TermCommandDescriptor * a, TermCommandDescriptor * b){
    uint8_t currPos = 0;
    //compare the lowercase ASCII values of each character in the command (They are alphabetically sorted)
    for(;currPos < a->commandLength && currPos < b->commandLength; currPos++){
        char letterA = toLowerCase(a->command[currPos]);
        char letterB = toLowerCase(b->command[currPos]);
        //if the letters are different we return 1 if a is smaller than b (they are correctly sorted) or zero if its the other way around
        if(letterA > letterB){
            return 1;
        }else if(letterB > letterA){
            return 0;
        }
    }
    
    //the two commands have identical letters for their entire length we check which one is longer (the shortest should come first)
    if(a->commandLength > b->commandLength){
        return 1;
    }else if(b->commandLength > a->commandLength){
        return 0;
    }else{
        //it might happen that a command is added twice (or two identical ones are added), in which case we just say they are sorted correctly and print an error in the console
        //TODO implement an alarm here
        //UART_print("WARNING: Found identical commands: \"%S\" and \"%S\"\r\n", a->command, b->command);        
        return 1;
    }
}

uint16_t toLowerCase(uint16_t c){
    if(c > 65 && c < 90){
        return c + 32;
    }
    
    switch(c){
        case 0xc39c: //Ü
            return 0xc3bc;  //ü
        case 0xc384: //Ä
            return 0xc3a4;  //ä
        case 0xc396:  //Ö
            return 0xc3b6;  //ö
        default:
            return c;
    }
}

void TERM_Box(TERMINAL_HANDLE * handle, uint8_t row1, uint8_t col1, uint8_t row2, uint8_t col2) {
	TERM_setCursorPos(handle, row1, col1);
	TERM_sendVT100Code(handle, _VT100_BACKGROUND_COLOR, _VT100_BLUE);
	ttprintf("\xE2\x95\x94"); //edge upper left
	int i = 0;
	for (i = 1; i < (col2 - col1); i++) {
		ttprintf("\xE2\x95\x90"); //=
	}
	ttprintf("\xE2\x95\x97"); //edge upper right
	for (i = 1; i < (row2 - row1); i++) {
		TERM_setCursorPos(handle, row1 + i, col1);
		ttprintf("\xE2\x95\x91"); //left ||
		TERM_setCursorPos(handle, row1 + i, col2);
		ttprintf("\xE2\x95\x91"); //right ||
	}
	TERM_setCursorPos(handle, row2, col1);
	ttprintf("\xE2\x95\x9A"); //edge lower left
	for (i = 1; i < (col2 - col1); i++) {
		ttprintf("\xE2\x95\x90"); //=
	}
	ttprintf("\xE2\x95\x9D"); //edge lower right
	TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
}


void TERM_setCursorPos(TERMINAL_HANDLE * handle, uint16_t row, uint16_t column){
    ttprintf(ESC_STR "[%i;%iH", row, column);
}

void TERM_sendVT100Code(TERMINAL_HANDLE * handle, uint16_t cmd, uint8_t var){
    switch(cmd){
        case _VT100_RESET:
            ttprintfEcho("%cc", 0x1b);
            break;
        case _VT100_CURSOR_BACK:
            ttprintfEcho("\x1b[D");
            break;
        case _VT100_CURSOR_FORWARD:
            ttprintfEcho("\x1b[C");
            break;
        case _VT100_CURSOR_POS1:
            ttprintfEcho("\x1b[H");
            break;
        case _VT100_CURSOR_END:
            ttprintfEcho("\x1b[F");
            break;
        case _VT100_FOREGROUND_COLOR:
            ttprintfEcho("\x1b[%dm", var+30);
            break;
        case _VT100_BACKGROUND_COLOR:
            ttprintfEcho("\x1b[%dm", var+40);
            break;
        case _VT100_RESET_ATTRIB:
            ttprintfEcho("\x1b[0m");
            break;
        case _VT100_BRIGHT:
            ttprintfEcho("\x1b[1m");
            break;
        case _VT100_DIM:
            ttprintfEcho("\x1b[2m");
            break;
        case _VT100_UNDERSCORE:
            ttprintfEcho("\x1b[4m");
            break;
        case _VT100_BLINK:
            ttprintfEcho("\x1b[5m");
            break;
        case _VT100_REVERSE:
            ttprintfEcho("\x1b[7m");
            break;
        case _VT100_HIDDEN:
            ttprintfEcho("\x1b[8m");
            break;
        case _VT100_ERASE_SCREEN:
            ttprintfEcho("\x1b[2J");
            break;
        case _VT100_ERASE_LINE:
            ttprintfEcho("\x1b[2K");
            break;
        case _VT100_FONT_G0:
            ttprintfEcho("\x1b(");
            break;
        case _VT100_FONT_G1:
            ttprintfEcho("\x1b)");
            break;
        case _VT100_WRAP_ON:
            ttprintfEcho("\x1b[7h");
            break;
        case _VT100_WRAP_OFF:
            ttprintfEcho("\x1b[7l");
            break;
        case _VT100_ERASE_LINE_END:
            ttprintfEcho("\x1b[K");
            break;
        case _VT100_CURSOR_BACK_BY:
            ttprintfEcho("\x1b[%dD", var);
            break;
        case _VT100_CURSOR_FORWARD_BY:
            ttprintfEcho("\x1b[%dC", var);
            break;
        case _VT100_CURSOR_SAVE_POSITION:
            ttprintfEcho("\x1b" "7");
            break;
        case _VT100_CURSOR_RESTORE_POSITION:
            ttprintfEcho("\x1b" "8");
            break;
        case _VT100_CURSOR_ENABLE:
            ttprintfEcho("\x1b[?25h");
            break;
        case _VT100_CURSOR_DISABLE:
            ttprintfEcho("\x1b[?25l");
            break;
        case _VT100_CURSOR_SET_COLUMN:
            ttprintfEcho(ESC_STR "[%i`", var);
            break;
        case _VT100_CLS:
            ttprintfEcho("\x1b[2J\033[1;1H");
            break;
        case _VT100_CURSOR_DOWN_BY:
            ttprintfEcho("\x1b[%dB", var);
            break;
            
    }
}

//returns a static pointer to the requested VT100 code, so it can be used in printf without needing any free() call afterwards
const char * TERM_getVT100Code(uint16_t cmd, uint8_t var){
    //INFO this is deprecated since all of the VT100 functions are handled by the terminal now
    switch(cmd){
        case _VT100_RESET:
            return "\x1b" "C";
            
        case _VT100_CURSOR_BACK:
            return "\x1b[D";
            
        case _VT100_CURSOR_FORWARD:
            return "\x1b[C";
            
        case _VT100_CURSOR_POS1:
            return "\x1b[H";
            
        case _VT100_CURSOR_END:
            return "\x1b[F";
        case _VT100_FOREGROUND_COLOR:
            switch(var){
                case 0:
                    return "\x1b[30m";
                case 1:
                    return "\x1b[31m";
                case 2:
                    return "\x1b[32m";
                case 3:
                    return "\x1b[33m";
                case 4:
                    return "\x1b[34m";
                case 5:
                    return "\x1b[35m";
                case 6:
                    return "\x1b[36m";
                case 7:
                    return "\x1b[37m";
                default:
                    return "\x1b[30m";
            }
            
        case _VT100_BACKGROUND_COLOR:
            switch(var){
                case 0:
                    return "\x1b[40m";
                case 1:
                    return "\x1b[41m";
                case 2:
                    return "\x1b[42m";
                case 3:
                    return "\x1b[43m";
                case 4:
                    return "\x1b[44m";
                case 5:
                    return "\x1b[45m";
                case 6:
                    return "\x1b[46m";
                case 7:
                    return "\x1b[47m";
                default:
                    return "\x1b[40m";
            }
            
        case _VT100_RESET_ATTRIB:
            return "\x1b[0m";
            
        case _VT100_BRIGHT:
            return "\x1b[1m";
            
        case _VT100_DIM:
            return "\x1b[2m";
            
        case _VT100_UNDERSCORE:
            return "\x1b[4m";
            
        case _VT100_BLINK:
            return "\x1b[5m";
            
        case _VT100_REVERSE:
            return "\x1b[7m";
            
        case _VT100_HIDDEN:
            return "\x1b[8m";
            
        case _VT100_ERASE_SCREEN:
            return "\x1b[2J";
            
        case _VT100_ERASE_LINE:
            return "\x1b[2K";
            
        case _VT100_FONT_G0:
            return "\x1b(";
            
        case _VT100_FONT_G1:
            return "\x1b)";
            
        case _VT100_WRAP_ON:
            return "\x1b[7h";
            
        case _VT100_WRAP_OFF:
            return "\x1b[71";
            
        case _VT100_ERASE_LINE_END:
            return "\x1b[K";
            
        case _VT100_CURSOR_ENABLE:
            return "\x1b[?25h";
            break;
        case _VT100_CURSOR_DISABLE:
            return "\x1b[?25l";
            break;
        case _VT100_CLS:
            return "\x1b[2J\033[1;1H";
            break;
            
    }
    return "";
}

void TERM_attachProgramm(TERMINAL_HANDLE * handle, TermProgram * prog){
    handle->currProgram = prog;
    handle->currProgram->inputStream = xStreamBufferCreate(TERM_PROG_BUFFER_SIZE,1);
}

void TERM_removeProgramm(TERMINAL_HANDLE * handle){
    handle->currProgram = NULL;
}

void TERM_killProgramm(TERMINAL_HANDLE * handle){
    uint8_t currArg = 0;
    uint8_t argCount = handle->currProgram->argCount;
    char ** args = handle->currProgram->args;
    for(;currArg<argCount; currArg++){
        vPortFree(args[currArg]);
    }
    
    TERM_sendVT100Code(handle, _VT100_RESET, 0); TERM_sendVT100Code(handle, _VT100_CURSOR_POS1, 0);
    TERM_printBootMessage(handle);
    
    TaskHandle_t task = handle->currProgram->task;
    vStreamBufferDelete(handle->currProgram->inputStream);
    vPortFree(handle->currProgram);
    handle->currProgram = NULL;
    vTaskDelete(task);
    while(1){
        vTaskDelay(100);
    }
}


#ifdef TERM_ENABLE_STARTUP_TEXT
#ifndef TERM_CUSTOM_STARTUP_TEXT
const char TERM_startupText1[] = "\r\n";
const char TERM_startupText2[] = "\r\n[M]olony [E]lectronic [S]peed [C]ontroller";
const char TERM_startupText3[] = "\r\n";
#endif
#endif   
