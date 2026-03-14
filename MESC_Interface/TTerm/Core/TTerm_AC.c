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

#include "FreeRTOS.h"
#include "task.h"
//#include "UART.h"
#include "TTerm/Core/include/TTerm.h"
#include "TTerm/Core/include/TTerm_cmd.h"
#include "TTerm/Core/include/TTerm_AC.h"

void TERM_addCommandAC(TermCommandDescriptor * cmd, TermAutoCompHandler ACH, void * ACParams){
    cmd->ACHandler = ACH;
    cmd->ACParams = ACParams;
}

uint8_t TERM_doAutoComplete(TERMINAL_HANDLE * handle){
    if(strnchr(handle->inputBuffer, ' ', handle->currBufferLength) != NULL){
        TermCommandDescriptor * cmd = TERM_findCMD(handle);
        if(cmd != NULL){
            if(cmd->ACHandler == 0){
                handle->currAutocompleteCount = 0;
                handle->autocompleteStart = 0;
                handle->autocompleteBufferLength = 0;
                return 0;
            }else{
                return (*cmd->ACHandler)(handle, cmd->ACParams);
            }
        } 
        handle->currAutocompleteCount = 0;
        handle->autocompleteStart = 0;
        handle->autocompleteBufferLength = 0;
        return 0;
    }else{
        handle->autocompleteBuffer = pvPortMalloc(handle->cmdListHead->commandLength * sizeof(char *));
        handle->currAutocompleteCount = 0;
        handle->autocompleteBufferLength = TERM_findMatchingCMDs(handle->inputBuffer, handle->currBufferLength, handle->autocompleteBuffer, handle->cmdListHead);
        handle->autocompleteStart = 0;
        return handle->autocompleteBufferLength;
    }
}

uint8_t TERM_findMatchingCMDs(char * currInput, uint8_t length, char ** buff, TermCommandDescriptor * cmdListHead){
    
    //TODO handle auto complete of parameters, for now we return if this is attempted
    if(strnchr(currInput, ' ', length) != NULL) return 0;
    //UART_print("scanning \"%s\" for matching cmds\r\n", currInput);
    
    uint8_t currPos = 0;
    uint8_t commandsFound = 0;
    TermCommandDescriptor * currCMD = cmdListHead->nextCmd;
    
    for(;currPos < cmdListHead->commandLength; currPos++){
        if(strncmp(currInput, currCMD->command, length) == 0){
            if(currCMD->commandLength >= length){
                buff[commandsFound] = (char*)currCMD->command;
                commandsFound ++;
                //UART_print("found %s (count is now %d)\r\n", TERM_cmdList[currPos]->command, commandsFound);
            }
        }else{
            if(commandsFound > 0) return commandsFound;
        }
        currCMD = currCMD->nextCmd;
    }
    return commandsFound;
}

uint8_t TERM_findLastArg(TERMINAL_HANDLE * handle, char * buff, uint8_t * lenBuff){
    uint8_t currPos = 0;
    unsigned quoteMark = 0;
    char * lastSpace = 0;
    for(;currPos<handle->currBufferLength; currPos++){
        switch(handle->inputBuffer[currPos]){
            case ' ':
                if(!quoteMark){
                    lastSpace = &handle->inputBuffer[currPos];
                }
                break;
                
            case '"':
                if(quoteMark){
                    quoteMark = 0;
                }else{
                    quoteMark = 1;
                    lastSpace = &handle->inputBuffer[currPos];
                }
                        
                break;
            default:
                break;
        }
    }
    
    *lenBuff = handle->currBufferLength - (lastSpace - handle->inputBuffer) - 1;
    memcpy(buff, lastSpace + 1, *lenBuff + 1);
    return (lastSpace - handle->inputBuffer) + ((*lastSpace == '"') ? 0 : 1);
}

uint8_t TERM_doListAC(AC_LIST_HEAD * head, char * currInput, uint8_t length, char ** buff){
    uint8_t currPos = 0;
    uint8_t commandsFound = 0;
    //UART_print("\r\nStart scan\r\n", buff[commandsFound], commandsFound+1);
    
    if(head->isConst){
        char ** strings = (char **) head->first;
        
        for(;currPos < head->elementCount; currPos++){
            //UART_print("\r\nchecking \"%s\"", strings[currPos]);
            if(strncmp(currInput, strings[currPos], length) == 0){
                if(strlen(strings[currPos]) >= length){
                    buff[commandsFound] = strings[currPos];
                    commandsFound ++;
                    //UART_print(" -> found match (count is now %d)", commandsFound);
                }
            }else{
                if(commandsFound > 0) return commandsFound;
            }
        }
        //UART_print("\r\n-----list done-----\r\n");
    }else{
        AC_LIST_ELEMENT * curr = head->first;
        for(;currPos < head->elementCount; currPos++){
            if(strncmp(currInput, curr->string, length) == 0){
                if(strlen(curr->string) >= length){
                    buff[commandsFound] = curr->string;
                    commandsFound ++;
                    //UART_print("found %s (count is now %d)\r\n", buff[commandsFound], commandsFound);
                }
            }else{
                if(commandsFound > 0) return commandsFound;
            }
            curr = ACL_getNext(curr);
            if(curr == 0) break;
        }
    }
    return commandsFound;
}

uint8_t ACL_defaultCompleter(TERMINAL_HANDLE * handle, void * params){
    if(params == 0){ 
        handle->autocompleteBufferLength = 0;
        return 0;
    }
    
    AC_LIST_HEAD * list = (AC_LIST_HEAD *) params;
    
    char * buff = pvPortMalloc(128);
    uint8_t len;
    memset(buff, 0, 128);
    handle->autocompleteStart = TERM_findLastArg(handle, buff, &len);
    
    //TODO use a reasonable size here
    handle->autocompleteBuffer = pvPortMalloc(list->elementCount * sizeof(char *));
    handle->currAutocompleteCount = 0;
    handle->autocompleteBufferLength = TERM_doListAC(list, buff, len, handle->autocompleteBuffer);
        
    vPortFree(buff);
    return handle->autocompleteBufferLength;
}

AC_LIST_ELEMENT * ACL_getNext(AC_LIST_ELEMENT * currElement){
    return currElement->next;
}

AC_LIST_HEAD * ACL_create(){
    AC_LIST_HEAD * ret = pvPortMalloc(sizeof(AC_LIST_HEAD));
    ret->elementCount = 0;
    ret->first = 0;
    ret->isConst = 0;
    return ret;
}

AC_LIST_HEAD * ACL_createConst(char ** strings, uint32_t count){
    AC_LIST_HEAD * ret = pvPortMalloc(sizeof(AC_LIST_HEAD));
    
    if(count == 0){  //autocount (requires "__LIST_END__" string)
        uint32_t currCount = 0;
        while(strcmp(strings[currCount], "__LIST_END__") != 0) currCount ++;
        ret->elementCount = currCount;
    }else{
        ret->elementCount = count;
    }
    
    ret->first = (AC_LIST_ELEMENT *) strings;
    ret->isConst = 1;
    return (AC_LIST_HEAD *) ret;
}

AC_LIST_ELEMENT * ACL_find(AC_LIST_HEAD * head, char * string){
    if(head->elementCount == 0) return 0;
    uint32_t currPos = 0;
    AC_LIST_ELEMENT * currComp = head->first;
    
    for(;currPos < head->elementCount; currPos++){
        if((strlen(currComp->string) == strlen(string)) && (strcmp(currComp->string, string) == 0)){
            return currComp;
        }
        if(currComp->next == 0) return 0;
        currComp = currComp->next;
    }
    return 0;
}

void ACL_add(AC_LIST_HEAD * head, char * string){
    if(head->isConst || ACL_find(head, string) != 0) return;
    
    if(head->elementCount == 0){
        AC_LIST_ELEMENT * newElement = pvPortMalloc(sizeof(AC_LIST_ELEMENT));
        newElement->string = string;
        newElement->next = 0;
        head->first = newElement;
        head->elementCount ++;
        return;
    }
    
    uint32_t currPos = 0;
    AC_LIST_ELEMENT ** lastComp = &head->first;
    AC_LIST_ELEMENT * currComp = head->first;
    
    while(currPos < head->elementCount){
        if(ACL_isSorted(currComp->string, string)){
            AC_LIST_ELEMENT * newElement = pvPortMalloc(sizeof(AC_LIST_ELEMENT));
            newElement->string = string;

            *lastComp = newElement;
            newElement->next = currComp;

            head->elementCount ++;
            
            break;
        }
        if(currComp->next == 0){
            AC_LIST_ELEMENT * newElement = pvPortMalloc(sizeof(AC_LIST_ELEMENT));
            newElement->string = string;
            newElement->string = string;
            newElement->next = currComp->next;
            currComp->next = newElement;
            head->elementCount ++;
            return;
        }
        lastComp = &currComp->next;
        currComp = currComp->next;
    }
}



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

unsigned ACL_isSorted(char * a, char * b){
    uint8_t currPos = 0;
    uint32_t aLength = strlen(a);
    uint32_t bLength = strlen(b);
    
    //compare the lowercase ASCII values of each character in the command (They are alphabetically sorted)
    for(;currPos < aLength && currPos < bLength; currPos++){
        char letterA = toLowerCase(a[currPos]);
        char letterB = toLowerCase(b[currPos]);
        //if the letters are different we return 1 if a is smaller than b (they are correctly sorted) or zero if its the other way around
        if(letterA > letterB){
            return 1;
        }else if(letterB > letterA){
            return 0;
        }
    }
    
    //the two commands have identical letters for their entire length we check which one is longer (the shortest should come first)
    if(aLength > bLength){
        return 1;
    }else if(bLength > aLength){
        return 0;
    }else{
        //it might happen that a command is added twice (or two identical ones are added), in which case we just say they are sorted correctly and print an error in the console
        //TODO implement an alarm here
        //UART_print("WARNING: Found identical commands: \"%S\" and \"%S\"\r\n", a, b);
        return 1;
    }
}
