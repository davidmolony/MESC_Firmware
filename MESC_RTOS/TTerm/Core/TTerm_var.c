/*
 * TTerm
 *
 * Copyright (c) 2022 Jens Kerrinnes
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

#include "TTerm/Core/include/TTerm_var.h"
#include "TTerm/Core/include/TTerm_fnv.h"
#include "TTerm/Core/include/TTerm.h"

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>


#define HEADER_START 	0xDEADBEEF
#define FOOTER_END   	0xDEADC0DE
#define HEADER_VERSION	0x00000001

const uint8_t null_data = 0;

bool is_init = false;
uint8_t password[16];


uint16_t toLower(uint16_t c){
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

static unsigned TERM_VAR_isSorted(TermVariableDescriptor * a, TermVariableDescriptor * b){
    uint8_t currPos = 0;
    //compare the lowercase ASCII values of each character in the command (They are alphabetically sorted)
    for(;currPos < a->nameLength && currPos < b->nameLength; currPos++){
        char letterA = toLower(a->name[currPos]);
        char letterB = toLower(b->name[currPos]);
        //if the letters are different we return 1 if a is smaller than b (they are correctly sorted) or zero if its the other way around
        if(letterA > letterB){
            return 1;
        }else if(letterB > letterA){
            return 0;
        }
    }

    //the two commands have identical letters for their entire length we check which one is longer (the shortest should come first)
    if(a->nameLength > b->nameLength){
        return 1;
    }else if(b->nameLength > a->nameLength){
        return 0;
    }else{
        //it might happen that a command is added twice (or two identical ones are added), in which case we just say they are sorted correctly and print an error in the console
        //TODO implement an alarm here
        //UART_print("WARNING: Found identical commands: \"%S\" and \"%S\"\r\n", a->command, b->command);
        return 1;
    }
}

TermVariableHandle * TERM_VAR_init(TERMINAL_HANDLE * handle, void * nvm_address, uint32_t nvm_size, nvm_clear nvm_clear, nvm_start_write nvm_start_write, nvm_write nvm_write, nvm_end_write nvm_end_write){

	TermVariableHandle * var = handle->varHandle;

	var->nvm_size = nvm_size;
	var->nvm_address = nvm_address;

	var->nvm_start_write = nvm_start_write;
	var->nvm_write = nvm_write;
	var->nvm_end_write = nvm_end_write;
	var->nvm_clear = nvm_clear;

	if(password[0]==0){
		handle->currPermissionLevel = 0;
	}

	if(is_init == false){
		TERM_addVarString(password, sizeof(password), 0, 0, "password", "Password for SU", VAR_ACCESS_RW, NULL, &TERM_varList);
		is_init = true;
	}

	return var;

}


void TERM_VAR_LIST_add(TermVariableDescriptor * item, TermVariableDescriptor * head){

    uint32_t currPos = 0;
    TermVariableDescriptor ** lastComp = &head->nextVar;
    TermVariableDescriptor * currComp = head->nextVar;

    while(currPos < head->nameLength){
        if(TERM_VAR_isSorted(currComp, item)){
            *lastComp = item;
            item->nextVar = currComp;
            head->nameLength ++;

            return;
        }
        if(currComp->nextVar == 0){
            item->nextVar = currComp->nextVar;
            currComp->nextVar = item;
            head->nameLength ++;
            return;
        }
        lastComp = &currComp->nextVar;
        currComp = currComp->nextVar;
    }

    item->nextVar = 0;
    *lastComp = item;
    head->nameLength ++;
}


TermVariableDescriptor * TERM_addVarUnsigned(void* variable, uint16_t typeSize, uint32_t min, uint32_t max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = min;
    newVAR->max_unsigned = max;
    newVAR->type = TERM_VARIABLE_UINT;
    newVAR->typeSize = typeSize;
    newVAR->name = name;
    newVAR->cb = cb;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

void TERM_setFlag(TermVariableDescriptor * desc, TermFlagType flag){
	desc->flags |= flag;
}

void TERM_clearFlag(TermVariableDescriptor * desc, TermFlagType flag){
	desc->flags &= ~flag;
}

bool TERM_check_protection(TermVariableDescriptor * desc, uint8_t level){

	uint32_t shifted_flag = desc->flags >> 28;

	if(level <= shifted_flag){
		return true;
	}else{
		return false;
	}

}

void TERM_set_protection(TermVariableDescriptor * desc, uint8_t level){
	uint32_t level_shifted = level << 28;
	desc->flags &= 0x0FFFFFFF;  //Clear all protection Flags
	desc->flags |= level_shifted;
}

TermVariableDescriptor * TERM_addVarSigned(void* variable, uint16_t typeSize, int32_t min, int32_t max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_signed = min;
    newVAR->max_signed = max;
    newVAR->type = TERM_VARIABLE_INT;
    newVAR->typeSize = typeSize;
    newVAR->name = name;
    newVAR->cb = cb;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarFloat(void* variable, uint16_t typeSize, float min, float max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_float = min;
    newVAR->max_float = max;
    newVAR->type = TERM_VARIABLE_FLOAT;
    newVAR->typeSize = typeSize;
    newVAR->name = name;
    newVAR->cb = cb;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarArrayFloat(void* variable, uint32_t size,  float min, float max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_float = min;
    newVAR->max_float = max;
    newVAR->type = TERM_VARIABLE_FLOAT_ARRAY;
    newVAR->typeSize = size;
    newVAR->name = name;
    newVAR->cb = cb;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarString(void* variable, uint16_t typeSize, uint32_t min, uint32_t max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = 0;
    newVAR->max_unsigned = 0;
    newVAR->type = TERM_VARIABLE_STRING;
    newVAR->typeSize = typeSize;
    newVAR->name = name;
    newVAR->cb = cb;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarChar(void* variable, uint16_t typeSize, uint32_t min, uint32_t max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = 0;
    newVAR->max_unsigned = 0;
    newVAR->type = TERM_VARIABLE_CHAR;
    newVAR->typeSize = 1;
    newVAR->name = name;
    newVAR->cb = cb;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarBool(void* variable, uint16_t typeSize, uint32_t min, uint32_t max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = 0;
    newVAR->max_unsigned = 1;
    newVAR->type = TERM_VARIABLE_BOOL;
    newVAR->typeSize = sizeof(bool);
    newVAR->name = name;
    newVAR->cb = cb;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

static uint8_t TERM_doVarListAC(TermVariableDescriptor * head, char * currInput, uint8_t length, char ** buff){
    uint8_t currPos = 0;
    uint8_t commandsFound = 0;
    //UART_print("\r\nStart scan\r\n", buff[commandsFound], commandsFound+1);

    TermVariableDescriptor * curr = head->nextVar;
	for(;currPos < head->nameLength; currPos++){
		if(strncmp(currInput, curr->name, length) == 0){
			if(strlen(curr->name) >= length){
				buff[commandsFound] = (char*)curr->name;
				commandsFound ++;
				//UART_print("found %s (count is now %d)\r\n", buff[commandsFound], commandsFound);
			}
		}else{
			if(commandsFound > 0) return commandsFound;
		}
		curr = curr->nextVar;
		if(curr == 0) break;
	}

    return commandsFound;
}

uint8_t TERM_varCompleter(TERMINAL_HANDLE * handle, void * params){
    if(params == 0){
        handle->autocompleteBufferLength = 0;
        return 0;
    }

    char * buff = pvPortMalloc(128);
    uint8_t len;
    memset(buff, 0, 128);
    handle->autocompleteStart = TERM_findLastArg(handle, buff, &len);

    TermVariableDescriptor * head = handle->varHandle->varListHead;

    //TODO use a reasonable size here
    handle->autocompleteBuffer = pvPortMalloc(head->nameLength * sizeof(char *));
    handle->currAutocompleteCount = 0;

    handle->autocompleteBufferLength = head->nameLength;
    handle->autocompleteBufferLength = TERM_doVarListAC(head, buff, len, handle->autocompleteBuffer);

    vPortFree(buff);
    return handle->autocompleteBufferLength;
}

static bool var_system_is_init(TERMINAL_HANDLE * handle){
	bool ret = true;
	TermVariableHandle * var = handle->varHandle;
	if(var->nvm_size == 0){
		ttprintf("Memory size not initialized\r\n");
		ret = false;
	}
	if(var->nvm_start_write == NULL || var->nvm_clear == NULL || var->nvm_end_write == NULL || var->nvm_write == NULL){
		ttprintf("Write functions not initialized\r\n");
		ret = false;
	}
	if(var->nvm_address == NULL){
		ttprintf("Memory ptr not initialized\r\n");
		ret = false;
	}

	return ret;
}


#define COL_A 3
#define COL_B 25
#define COL_C 45
#define COL_D 56
#define COL_E 67

static void print_var_header(TERMINAL_HANDLE * handle){
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_A);
	ttprintf("Parameter");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
	ttprintf("| Value");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_C);
	ttprintf("| Min");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_D);
	ttprintf("| Max");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_E);
	ttprintf("| Description\r\n");
}

static void print_var_header_update(TERMINAL_HANDLE * handle){
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_A);
	ttprintf("Parameter");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
	ttprintf("| Value");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_C);
	ttprintf("| Min");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_D);
	ttprintf("| Max");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_E);
	ttprintf("| Changes\r\n");
}


uint32_t TERM_var2str(TERMINAL_HANDLE * handle, TermVariableDescriptor * var, char * buffer, int32_t len ){

    uint32_t u_temp_buffer = 0;
    int32_t i_temp_buffer = 0;
    //float f_buffer;
    uint32_t ret = 0;

	switch (var->type){
	case TERM_VARIABLE_UINT:
		switch (var->typeSize){
		case 1:
			u_temp_buffer = *(uint8_t*)var->variable;
			break;
		case 2:
			u_temp_buffer = *(uint16_t*)var->variable;
			break;
		case 4:
			u_temp_buffer = *(uint32_t*)var->variable;
			break;
		}

		ret = snprintf(buffer, len, "%lu", u_temp_buffer);
		break;
	case TERM_VARIABLE_INT:
		switch (var->typeSize){
		case 1:
			i_temp_buffer = *(int8_t*)var->variable;
			break;
		case 2:
			i_temp_buffer = *(int16_t*)var->variable;
			break;
		case 4:
			i_temp_buffer = *(int32_t*)var->variable;
			break;
		}

		ret = snprintf(buffer, len, "%li", i_temp_buffer);

		break;
	case TERM_VARIABLE_FLOAT:

		ret = snprintf(buffer, len, "%.3f", *(float*)var->variable);

		break;
	case TERM_VARIABLE_FLOAT_ARRAY:
//		if(flag == HELPER_FLAG_DETAIL){
//			for(uint32_t cnt=0;cnt<(var->typeSize / sizeof(float));cnt++){
//				f_buffer = ((float*)var->variable)[cnt];
//				TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
//				ttprintf("\033[37m| [%u] \033[32m%f", cnt , f_buffer);
//				if(cnt<(var->typeSize / sizeof(float)-1)){
//					ttprintf("\r\n");
//				}
//			}
//		}else{
//			TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
//			ttprintf("\033[37m| \033[32mArray[%u]", (var->typeSize / sizeof(float)));
//		}
//
		break;
	case TERM_VARIABLE_INT_ARRAY:
		//TODO
		break;
	case TERM_VARIABLE_UINT_ARRAY:
		//TODO
		break;
	case TERM_VARIABLE_CHAR:

		ret = snprintf(buffer, len, "%c", *(char*)var->variable);

		break;
	case TERM_VARIABLE_STRING:

		ret = snprintf(buffer, len, "%s", (char*)var->variable);

		break;
	case TERM_VARIABLE_BOOL:

		ret = snprintf(buffer, len, "%s", *(bool*)var->variable ? "true" : "false");

		break;
	}
	return ret;
}


void print_var_helperfunc(TERMINAL_HANDLE * handle, TermVariableDescriptor * var, HelperFlagType flag ){

    uint32_t u_temp_buffer=0;
    int32_t i_temp_buffer=0;
    float f_buffer;

	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_A);
	ttprintf("\033[36m%s", var->name);

	switch (var->type){
	case TERM_VARIABLE_UINT:
		switch (var->typeSize){
		case 1:
			u_temp_buffer = *(uint8_t*)var->variable;
			break;
		case 2:
			u_temp_buffer = *(uint16_t*)var->variable;
			break;
		case 4:
			u_temp_buffer = *(uint32_t*)var->variable;
			break;
		}

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%u", u_temp_buffer);

		break;
	case TERM_VARIABLE_INT:
		switch (var->typeSize){
		case 1:
			i_temp_buffer = *(int8_t*)var->variable;
			break;
		case 2:
			i_temp_buffer = *(int16_t*)var->variable;
			break;
		case 4:
			i_temp_buffer = *(int32_t*)var->variable;
			break;
		}

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%i", i_temp_buffer);

		break;
	case TERM_VARIABLE_FLOAT:

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%f", (double)*(float*)var->variable);

		break;
	case TERM_VARIABLE_FLOAT_ARRAY:
		if(flag == HELPER_FLAG_DETAIL){
			for(uint32_t cnt=0;cnt<(var->typeSize / sizeof(float));cnt++){
				f_buffer = ((float*)var->variable)[cnt];
				TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
				ttprintf("\033[37m| [%u] \033[32m%f", cnt , (double)f_buffer);
				if(cnt<(var->typeSize / sizeof(float)-1)){
					ttprintf("\r\n");
				}
			}
		}else{
			TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
			ttprintf("\033[37m| \033[32mArray[%u]", (var->typeSize / sizeof(float)));
		}

		break;
	case TERM_VARIABLE_INT_ARRAY:
		//TODO
		break;
	case TERM_VARIABLE_UINT_ARRAY:
		//TODO
		break;
	case TERM_VARIABLE_CHAR:

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%c", *(char*)var->variable);

		break;
	case TERM_VARIABLE_STRING:

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%s", (char*)var->variable);

		break;
	case TERM_VARIABLE_BOOL:

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%s", *(bool*)var->variable ? "true" : "false");

		break;
	}

	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_C);
	if(flag != HELPER_FLAG_FLASH){
		switch (var->type){
			case TERM_VARIABLE_UINT:
				ttprintf("\033[37m| %u", var->min_unsigned);
				break;
			case TERM_VARIABLE_INT:
				ttprintf("\033[37m| %i", var->min_signed);
				break;
			case TERM_VARIABLE_FLOAT:
			case TERM_VARIABLE_FLOAT_ARRAY:
				ttprintf("\033[37m| %.2f", (double)var->min_float);
				break;
			case TERM_VARIABLE_BOOL:
				ttprintf("\033[37m| false");
				break;
			default:
				//ttprintf("\033[37m| -");
				break;
		}
	}else{
		if(var->type != TERM_VARIABLE_STRING){
			ttprintf("\033[37m| -");
		}
	}

	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_D);
	if(flag != HELPER_FLAG_FLASH){
		switch (var->type){
			case TERM_VARIABLE_UINT:
				ttprintf("\033[37m| %u", var->max_unsigned);
				break;
			case TERM_VARIABLE_INT:
				ttprintf("\033[37m| %i", var->max_signed);
				break;
			case TERM_VARIABLE_FLOAT:
			case TERM_VARIABLE_FLOAT_ARRAY:
				ttprintf("\033[37m| %.2f", (double)var->max_float);
				break;
			case TERM_VARIABLE_BOOL:
				ttprintf("\033[37m| true");
				break;
			default:
				//ttprintf("\033[37m| -");
				break;
		}
	}else{
		if(var->type != TERM_VARIABLE_STRING){
			ttprintf("\033[37m| -");
		}
	}
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_E);
	if(flag != HELPER_FLAG_FLASH){
		if(var->variableDescription){
			ttprintf("\033[37m| %s\r\n", var->variableDescription);
		}else{
			ttprintf("\033[37m| -\r\n");
		}
	}else{
			ttprintf("\033[37m| ");
	}

}

uint8_t CMD_varList(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	HelperFlagType flag = HELPER_FLAG_DEFAULT;

	if(argCount && strcmp("-d",args[argCount-1])==0){
		flag = HELPER_FLAG_DETAIL;
		argCount--;
	}

	uint8_t currPos = 0;

	print_var_header(handle);

	TermVariableDescriptor * head = handle->varHandle->varListHead;
	TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;
    for(;currPos < head->nameLength; currPos++){
    	if(TERM_check_protection(currVar, handle->currPermissionLevel)){
			if(argCount && args[0] != NULL){
				if(strstr(currVar->name, args[0])){
					print_var_helperfunc(handle, currVar, flag);
				}
			}else{
				print_var_helperfunc(handle, currVar, flag);
			}
    	}
        currVar = currVar->nextVar;
    }

	return TERM_CMD_EXIT_SUCCESS;
}

static float getMul(char mul){
	switch(mul){
		case 'u':
			return 1e-6f;
		case 'm':
			return 1e-3f;
		case 'k':
			return 1e3f;
		case 'M':
			return 1e6f;
		default:
			return 1.0f;
	}
}

static bool set_value(TermVariableDescriptor * var, char* value){
	bool truncated = false;
    uint32_t u_temp_buffer=0;
    int32_t i_temp_buffer=0;
    float f_temp_buffer=0.0f;

    uint32_t index=0;
    char* next_char=NULL;
    uint32_t len = strlen(value);
    char* mul = NULL;

	switch (var->type){
	case TERM_VARIABLE_UINT:
		u_temp_buffer = strtoul(value, NULL, 0);

		if(u_temp_buffer < var->min_unsigned){
			u_temp_buffer = var->min_unsigned;
			truncated = true;
		}else if(u_temp_buffer > var->max_unsigned){
			u_temp_buffer = var->max_unsigned;
			truncated = true;
		}

		switch (var->typeSize){
		case 1:
			*(uint8_t*)var->variable = u_temp_buffer;
			break;
		case 2:
			*(uint16_t*)var->variable = u_temp_buffer;
			break;
		case 4:
			*(uint32_t*)var->variable = u_temp_buffer;
			break;
		}
		break;
	case TERM_VARIABLE_INT:
		i_temp_buffer = strtol(value, NULL, 0);

		if(i_temp_buffer < var->min_signed){
			i_temp_buffer = var->min_signed;
			truncated = true;
		}else if(i_temp_buffer > var->max_signed){
			i_temp_buffer = var->max_signed;
			truncated = true;
		}

		switch (var->typeSize){
		case 1:
			*(int8_t*)var->variable = i_temp_buffer;
			break;
		case 2:
			*(int16_t*)var->variable = i_temp_buffer;
			break;
		case 4:
			*(int32_t*)var->variable = i_temp_buffer;
			break;
		}
		break;
	case TERM_VARIABLE_FLOAT:
		f_temp_buffer = strtof(value, &mul);

		if(mul != NULL){
			f_temp_buffer *= getMul(*mul);
		}

		if(f_temp_buffer < var->min_float){
			f_temp_buffer = var->min_float;
			truncated = true;
		}else if(f_temp_buffer > var->max_float){
			f_temp_buffer = var->max_float;
			truncated = true;
		}

		*(float*)var->variable = f_temp_buffer;

		break;
	case TERM_VARIABLE_FLOAT_ARRAY:
		next_char = strstr(value, "]")+1;
		if(value[0]=='[') value++;
		index = strtoul(value, _NULL, 10);
		if(next_char==NULL) break;
		if(index>var->typeSize / sizeof(float)-1) break;

		f_temp_buffer = strtof(next_char, &mul);

		if(mul != NULL){
			f_temp_buffer *= getMul(*mul);
		}

		if(f_temp_buffer < var->min_float){
			f_temp_buffer = var->min_float;
			truncated = true;
		}else if(f_temp_buffer > var->max_float){
			f_temp_buffer = var->max_float;
			truncated = true;
		}

		((float*)var->variable)[index] = f_temp_buffer;

		break;
	case TERM_VARIABLE_INT_ARRAY:
		//TODO
		break;
	case TERM_VARIABLE_UINT_ARRAY:
		//TODO
		break;
	case TERM_VARIABLE_CHAR:
		*(char*)var->variable = value[0];
		break;
	case TERM_VARIABLE_STRING:
		strncpy((char*)var->variable, value, var->typeSize);
		if(strlen(value) > var->typeSize-1){
			truncated=true;
		}
		break;
	case TERM_VARIABLE_BOOL:
		for(uint32_t i=0;i<len;i++){
			value[i] = toLower(value[i]);
		}
		if(strcmp(value, "true")==0) *(bool*)var->variable = true;
		if(strcmp(value, "false")==0) *(bool*)var->variable = false;
		if(strcmp(value, "1")==0) *(bool*)var->variable = true;
		if(strcmp(value, "0")==0) *(bool*)var->variable = false;
		break;
	}
	return truncated;
}

uint8_t CMD_varSet(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	uint32_t currPos = 0;
	if(argCount<2){
		ttprintf("Usage: set [name] [value]");
		return TERM_CMD_EXIT_SUCCESS;
	}

	TermVariableDescriptor * head = handle->varHandle->varListHead;
	TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;

    for(;currPos < head->nameLength; currPos++){
    	if(strcmp(args[0], currVar->name)==0){
    		if(currVar->rw & VAR_ACCESS_W){
    			if(TERM_check_protection(currVar, handle->currPermissionLevel) == false){
    				ttprintf("No permission\r\n");
    				return TERM_CMD_EXIT_SUCCESS;
    			}

				bool truncated = set_value(currVar, args[1]);
				if(currVar->cb != NULL){
					currVar->cb(currVar);
				}
				print_var_header(handle);
				print_var_helperfunc(handle, currVar, HELPER_FLAG_DETAIL);
				if(truncated){
					TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
					TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_RED);
					ttprintf("  Value truncated\r\n");
					TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
				}
				return TERM_CMD_EXIT_SUCCESS;
    		}else{
    			ttprintf("  Variable not writable\r\n");
    			return TERM_CMD_EXIT_SUCCESS;
    		}
    	}

        currVar = currVar->nextVar;
    }

	return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_varChown(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	uint32_t currPos = 0;
	if(argCount<2){
		ttprintf("Usage: chown [name] [level]");
		return TERM_CMD_EXIT_SUCCESS;
	}

	TermVariableDescriptor * head = handle->varHandle->varListHead;
	TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;

    for(;currPos < head->nameLength; currPos++){
    	if(strcmp(args[0], currVar->name)==0){
    			if(TERM_check_protection(currVar, handle->currPermissionLevel) == false){
    				ttprintf(" No permission\r\n");
    				return TERM_CMD_EXIT_SUCCESS;
    			}
    			uint8_t permission_level = strtoul(args[1], NULL, 0);
    			if(permission_level>15){
    				ttprintf(" Only permission levels <16 are allowed\r\n");
    				return TERM_CMD_EXIT_SUCCESS;
    			}
    			TERM_set_protection(currVar, permission_level);
    			ttprintf(" Changed permission to %u\r\n", permission_level);

				return TERM_CMD_EXIT_SUCCESS;
    	}

        currVar = currVar->nextVar;
    }

	return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_su(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	for(int i=0;i<argCount;i++){
		if(strcmp(args[i], "-e")==0){
			handle->currPermissionLevel = handle->userPermissionLevel;
			ttprintf("You are back to normal\r\n");
		}
		if(strcmp(args[i], "-g")==0){
			if(i+1 < argCount){
				if(strcmp(args[i+1],(char*)password) == 0){
					handle->currPermissionLevel = 0;
					ttprintf(" Godmode\r\n");
				}else{
					ttprintf(" Password wrong\r\n");
				}
			}else{
				if(password[0] == 0){
					handle->currPermissionLevel = 0;
					ttprintf(" Godmode\r\n");
				}
			}
		}
		if(strcmp(args[i], "-?")==0){
			ttprintf("Usage: su [flags]\r\n");
			ttprintf("\t -e\t Exit to default permission\r\n");
			ttprintf("\t -g [password]\t Godmode\r\n");
			return TERM_CMD_EXIT_SUCCESS;
		}
	}
	return TERM_CMD_EXIT_SUCCESS;
}


typedef struct _FlashHeader_ FlashHeader;
typedef struct _FlashFooter_ FlashFooter;
typedef struct _FlashVariable_ FlashVariable;

struct _FlashHeader_{
	uint32_t start;
	uint32_t num_entries;
	uint32_t size;
	uint32_t version;
	uint32_t revision;
#ifdef ALLIGNED_DOUBLE_WORD
	uint32_t padding;
#endif
} __attribute__((packed));

struct _FlashFooter_{
	uint32_t end;
	uint32_t crc;
} __attribute__((packed));

struct _FlashVariable_{
    void * variable;
    TermVariableType type;
    uint16_t typeSize;
    const char * name;
    uint32_t nameLength;
    uint32_t flags;
    FlashVariable * nextVar;
#ifdef ALLIGNED_DOUBLE_WORD
	uint8_t padding;
#endif
} __attribute__((packed));


static uint8_t * find_next_free_memory(uint8_t * address, uint32_t storage_size){
	//Find last active header
	FlashHeader * header_ptr;
	header_ptr = (FlashHeader*)address;
	uint8_t * header_section = address;
	while(header_ptr->start == HEADER_START && header_section < address + storage_size){
		header_section += header_ptr->size;
		header_ptr = (FlashHeader*)header_section;
	}
	return header_section;
}

static uint8_t * find_last_active_header(bool find_revision, uint32_t revision, uint8_t * address, uint32_t storage_size){
	//Find last active header
	FlashHeader * header_ptr;
	header_ptr = (FlashHeader*)address;
	uint32_t size_last=0;
	uint8_t * header_section = address;
	while(header_ptr->start == HEADER_START && header_section < address + storage_size){
		header_section += header_ptr->size;
		size_last = header_ptr->size;
		if(find_revision && header_ptr->revision == revision){
			return (uint8_t*)header_ptr;
		}
		header_ptr = (FlashHeader*)header_section;
	}
	if(find_revision){
		return NULL;
	}else{
		return header_section - size_last;
	}
}

static uint8_t * print_headers(TERMINAL_HANDLE * handle, uint8_t * address, uint32_t storage_size){
	//Find last active header
	FlashHeader * header_ptr;
	header_ptr = (FlashHeader*)address;
	uint32_t size_last=0;
	uint8_t * header_section = address;
	while(header_ptr->start == HEADER_START && header_section < address + storage_size){
		FlashFooter * footer = (FlashFooter *)((uint8_t*)header_ptr + header_ptr->size - sizeof(FlashFooter));
		ttprintf("Revision: %u Size: %u CRC: %08x \r\n", header_ptr->revision, header_ptr->size, footer->crc);
		header_section += header_ptr->size;
		size_last = header_ptr->size;
		header_ptr = (FlashHeader*)header_section;
	}
	return header_section - size_last;
}

uint32_t get_padding(uint32_t num, uint32_t allignement){

	uint32_t remainder = num % allignement;
	return allignement - remainder;

}

uint32_t validate(TERMINAL_HANDLE * handle, FlashHeader * header){
	TermVariableHandle * var = handle->varHandle;

	FlashVariable * FlashVar = (FlashVariable*)(header + 1);
	FlashVariable * currFlashVar = FlashVar;
	FlashFooter * footer = (FlashFooter *)((uint8_t*)header + header->size - sizeof(FlashFooter));

	uint32_t crc = TTERM_fnv1a_init();
	crc = TTERM_fnv1a_process_data(crc, header, sizeof(FlashHeader));

	//Checking CRC
	for(uint32_t currPos=0;currPos < header->num_entries; currPos++){
		if(currFlashVar < (FlashVariable *)var->nvm_address || currFlashVar > (FlashVariable *)(var->nvm_address + var->nvm_size)){
			return 0;
		}
		crc = TTERM_fnv1a_process_data(crc, currFlashVar, sizeof(FlashVariable));
		currFlashVar = currFlashVar->nextVar;
	}

	currFlashVar = FlashVar;
	for(uint32_t currPos=0;currPos < header->num_entries; currPos++){
		uint32_t size = currFlashVar->nameLength + 1 + currFlashVar->typeSize;  //Include variable data which follows the name

#ifdef ALLIGNED_DOUBLE_WORD
		size += get_padding(size, 8);
#endif
		crc = TTERM_fnv1a_process_data(crc, currFlashVar->name, size);
		currFlashVar = currFlashVar->nextVar;
	}

	return TTERM_fnv1a_process_data(crc, footer, sizeof(FlashFooter) - sizeof(uint32_t));
}



uint8_t CMD_varSave(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	if(var_system_is_init(handle) == false) return TERM_CMD_EXIT_SUCCESS;


	TermVariableHandle * var = handle->varHandle;

	FlashHeader header;
	FlashFooter footer;

	footer.crc = TTERM_fnv1a_init();

	uint32_t written=0;
	uint32_t largest_data=0;


	uint32_t currPos = 0;
	TermVariableDescriptor * head = var->varListHead;
	TermVariableDescriptor * currVar = var->varListHead->nextVar;


	uint32_t storage_size = var->nvm_size;
	uint8_t * address = var->nvm_address;
	uint8_t * header_section = address;

	if(argCount && strcmp("-d",args[argCount-1])==0){
		ttprintf("Erasing flash...\r\n");
		var->nvm_clear(address, storage_size);
	}

	if(argCount && strcmp("-h",args[argCount-1])==0){
		ttprintf("Erasing flash...\r\n");
		var->nvm_clear(address, storage_size);
		return TERM_CMD_EXIT_SUCCESS;
	}

	//Determine how much memory is needed
	uint32_t n_bytes = sizeof(FlashHeader) + (sizeof(FlashVariable) * head->nameLength) + sizeof(FlashFooter);  //nameLength is the number of vars here
	for(;currPos < head->nameLength; currPos++){
		uint32_t bytes = 0;
		bytes += (currVar->nameLength+1);
		bytes += currVar->typeSize;

#ifdef ALLIGNED_DOUBLE_WORD
		bytes += get_padding(bytes, 8);
#endif
		n_bytes += bytes;

		if(largest_data < bytes) largest_data = bytes;
		currVar = currVar->nextVar;
	}

	if(n_bytes > storage_size){
		ttprintf("Memory overflow by %u bytes\r\n", n_bytes - storage_size);
		return TERM_CMD_EXIT_SUCCESS;
	}

	//Find last active header and increment revision
	FlashHeader * last_header = (FlashHeader*)find_last_active_header(false, 0, address, storage_size);
	handle->varHandle->nvm_revision = last_header->revision + 1;

	header_section = find_next_free_memory(address, storage_size);

	//Check if new dataset fits into memory
	if(header_section + n_bytes > address + storage_size){
		header_section = address;
		ttprintf("Memory full, erasing flash...\r\n");
		var->nvm_clear(address, storage_size);
	}

	FlashVariable * var_section 	 = (FlashVariable *)(header_section + sizeof(FlashHeader));
	uint8_t * data_section 	 = header_section + sizeof(FlashHeader) + (sizeof(FlashVariable) * head->nameLength);


	header.start = HEADER_START;
	header.num_entries = head->nameLength;
	header.version = HEADER_VERSION;
	header.size = n_bytes;
	header.revision = handle->varHandle->nvm_revision;

	written += var->nvm_start_write(header_section, &header, sizeof(header));
	footer.crc = TTERM_fnv1a_process_data(footer.crc, &header, sizeof(header));

	FlashVariable temp;
	memset(&temp,0,sizeof(FlashVariable));

	currPos = 0;
	currVar = head->nextVar;
	uint8_t * currData = data_section;

	//Write descriptors
	for(;currPos < head->nameLength; currPos++){

		uint32_t nameLengthWNull = currVar->nameLength+1;

		temp.nameLength = currVar->nameLength;
		temp.flags = currVar->flags;
		temp.type = currVar->type;
		temp.typeSize = currVar->typeSize;
		temp.name = (const char *)currData;
		temp.variable = currData + nameLengthWNull;

		if(currVar->nextVar){
			temp.nextVar = var_section + 1;
		}else{
			temp.nextVar = 0;
		}

		written += var->nvm_write(var_section, &temp, sizeof(FlashVariable));
		footer.crc = TTERM_fnv1a_process_data(footer.crc, &temp, sizeof(FlashVariable));

		//Calculate new data section pointers
		uint32_t bytes = nameLengthWNull;
		bytes += currVar->typeSize;
#ifdef ALLIGNED_DOUBLE_WORD
		bytes += get_padding(bytes, 8);
#endif
		currData += bytes;

		var_section++;
		currVar = currVar->nextVar;
	}

	//Write data
	currPos = 0;
	currVar = head->nextVar;
	currData = data_section;

	//Allocate data workingcopy
	uint8_t * data_cpy = pvPortMalloc(largest_data);
	if(data_cpy == NULL){
		ttprintf("Cannot allocate data copy\r\n");
		return TERM_CMD_EXIT_SUCCESS;
	}

	for(;currPos < head->nameLength; currPos++){
		memset(data_cpy,0,largest_data);

		uint32_t nameLengthWNull = currVar->nameLength+1;

		memcpy(data_cpy, currVar->name, nameLengthWNull);
		memcpy(data_cpy + nameLengthWNull, currVar->variable, currVar->typeSize);

		uint32_t bytes_to_write = nameLengthWNull + currVar->typeSize;

#ifdef ALLIGNED_DOUBLE_WORD
		bytes_to_write += get_padding(bytes_to_write, 8);
#endif

		//written += var->nvm_write(currData, (uint8_t*)currVar->name, nameLengthWNull);
		footer.crc = TTERM_fnv1a_process_data(footer.crc, data_cpy, bytes_to_write);
		//footer.crc = TTERM_fnv1a_process_data(footer.crc, data_cpy + nameLengthWNull, currVar->typeSize);

		written += var->nvm_write(currData, data_cpy, bytes_to_write);

		currData += bytes_to_write;

		currVar = currVar->nextVar;
	}

	vPortFree(data_cpy);


	footer.end = FOOTER_END;

	footer.crc = TTERM_fnv1a_process_data(footer.crc, &footer, sizeof(FlashFooter)-sizeof(uint32_t));  //Dont CRC the crc field
	written += var->nvm_end_write(currData, &footer, sizeof(FlashFooter));

	uint32_t crc = validate(handle, (FlashHeader*)header_section);

	if(footer.crc != crc){
		ttprintf("CRC mismatch Flash: %08x Saved: %08x\r\n", footer.crc, crc);
		return TERM_CMD_EXIT_SUCCESS;
	}else{
		ttprintf("Validating flash... ok\r\n", footer.crc, crc);
	}

	ttprintf("Saved %u variables in %u bytes with CRC: %08x\r\n", head->nameLength, n_bytes, footer.crc);

	return TERM_CMD_EXIT_SUCCESS;
}

static void print_var_flash(TERMINAL_HANDLE * handle, FlashVariable * flashVar){
	TermVariableDescriptor var;
	var.name = flashVar->name;
	var.nameLength = flashVar->nameLength;
	var.type = flashVar->type;
	var.typeSize = flashVar->typeSize;
	var.variable = flashVar->variable;
	print_var_helperfunc(handle, &var, HELPER_FLAG_FLASH);
}


uint8_t CMD_varLoad(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	if(var_system_is_init(handle) == false) return TERM_CMD_EXIT_SUCCESS;

	bool show_only = false;
	bool load_revision = false;
	uint32_t revision_to_load = 0;
	char * var_to_load = NULL;

	uint8_t * address = handle->varHandle->nvm_address;
	uint32_t storage_size = handle->varHandle->nvm_size;

	for(int i=0;i<argCount;i++){
		if(strcmp(args[i], "-s")==0){
			show_only = true;
		}
		if(strcmp(args[i], "-h")==0){
			print_headers(handle, address, storage_size);
			return TERM_CMD_EXIT_SUCCESS;
		}
		if(strcmp(args[i], "-r")==0){
			if(i+1 < argCount){
				load_revision = true;
				revision_to_load = strtoul(args[i+1], NULL, 10);
			}
		}
		if(strcmp(args[i], "-v")==0){
			if(i+1 < argCount){
				var_to_load = args[i+1];
			}
		}
		if(strcmp(args[i], "-?")==0){
			ttprintf("Usage: load [flags]\r\n");
			ttprintf("\t -s\t Show only\r\n");
			ttprintf("\t -h\t Show headers\r\n");
			ttprintf("\t -r\t Load specific revision\r\n");
			ttprintf("\t -v\t Load specific variable\r\n");
			return TERM_CMD_EXIT_SUCCESS;
		}
	}

	FlashHeader * header = (FlashHeader*)find_last_active_header(load_revision, revision_to_load, address, storage_size);

	if(header == NULL || header->start != HEADER_START || header->version != HEADER_VERSION){
			ttprintf("No dataset found\r\n");
		return TERM_CMD_EXIT_SUCCESS;
	}
	FlashFooter * footer = (FlashFooter *)((uint8_t*)header + header->size - sizeof(FlashFooter));
	if(footer->end != FOOTER_END){
		ttprintf("Unexpected footer\r\n");
		return TERM_CMD_EXIT_SUCCESS;
	}

	uint32_t currPos = 0;
	FlashVariable * FlashVar = (FlashVariable*)(header + 1);
	FlashVariable * currFlashVar = FlashVar;

	if(header > (FlashHeader *)(address + storage_size) || header < (FlashHeader *)address){
		ttprintf("Header out of bounds\r\n");
		return TERM_CMD_EXIT_SUCCESS;
	}

	uint32_t crc = validate(handle, header);

	if(footer->crc != crc){
		ttprintf("CRC mismatch Flash: %08x Loaded: %08x\r\n", footer->crc, crc);
		return TERM_CMD_EXIT_ERROR;
	}

	currPos = 0;
	currFlashVar = FlashVar;
	print_var_header_update(handle);

	if(show_only == false || load_revision == true){
		handle->varHandle->nvm_revision = header->revision;
	}
	for(;currPos < header->num_entries; currPos++){
		if(var_to_load != NULL){
			if(strcmp(currFlashVar->name, var_to_load) != 0){
				currFlashVar = currFlashVar->nextVar;
				continue;
			}
		}


		print_var_flash(handle, currFlashVar);

		uint32_t currUpdatePos = 0;
		bool found_var=false;

		TermVariableDescriptor * head = handle->varHandle->varListHead;
		TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;

		for(;currUpdatePos < head->nameLength; currUpdatePos++){
			if(strcmp(currFlashVar->name, currVar->name)==0){
				currVar->flags=currFlashVar->flags;
				if(currFlashVar->type == currVar->type && currFlashVar->typeSize == currVar->typeSize){
					if((currVar->rw & VAR_ACCESS_W)){
						if(show_only==false && memcmp(currVar->variable, currFlashVar->variable, currVar->typeSize) != 0){
							memcpy(currVar->variable, currFlashVar->variable, currVar->typeSize);
							ttprintf("Updated value from flash\r\n");
						}else{
							ttprintf("-\r\n");
						}
					}else{
						ttprintf("Not writable\r\n");
					}
				}else{
					ttprintf("Type or size mismatch\r\n");
				}
				found_var = true;
			}
			currVar = currVar->nextVar;
		}
		if(found_var==false){
			ttprintf("Cannot find variable in firmware\r\n");
		}


		currFlashVar = currFlashVar->nextVar;
	}

	return TERM_CMD_EXIT_SUCCESS;

}

