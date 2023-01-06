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


#define HEADER_START 	0xDEADBEEF
#define FOOTER_END   	0xDEADC0DE
#define HEADER_VERSION	0x00000001


typedef enum {
    HELPER_FLAG_FLASH,
	HELPER_FLAG_DETAIL,
	HELPER_FLAG_DEFAULT,
} HelperFlagType;


uint16_t toLower(uint16_t c){
    if(c > 65 && c < 90){
        return c + 32;
    }

    switch(c){
        case 'Ü':
            return 'ü';
        case 'Ä':
            return 'ä';
        case 'Ö':
            return 'ö';
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


//TermVariableDescriptor * TERM_addVar(void* variable, TermVariableType type, uint16_t typeSize, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
//    //if(head == NULL) head = TERM_defaultList;
//
//    if(head->nameLength == 0xff) return 0;
//
//    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));
//
//    newVAR->variable = variable;
//    newVAR->type = type;
//    newVAR->typeSize = typeSize;
//    newVAR->name = name;
//    newVAR->nameLength = strlen(name);
//    newVAR->variableDescription = description;
//    newVAR->rw = rw;
//
//    TERM_VAR_LIST_add(newVAR, head);
//    return newVAR;
//}

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

TermVariableDescriptor * TERM_addVarFloat(void* variable, float min, float max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_float = min;
    newVAR->max_float = max;
    newVAR->type = TERM_VARIABLE_FLOAT;
    newVAR->typeSize = sizeof(float);
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

TermVariableDescriptor * TERM_addVarString(void* variable, uint16_t typeSize, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head){
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

TermVariableDescriptor * TERM_addVarChar(void* variable, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head){
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

TermVariableDescriptor * TERM_addVarBool(void* variable, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head){
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
#define COL_B 15
#define COL_C 35
#define COL_D 46
#define COL_E 57

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

static void print_var_helperfunc(TERMINAL_HANDLE * handle, TermVariableDescriptor * var, HelperFlagType flag ){

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
		ttprintf("\033[37m| \033[32m%f", *(float*)var->variable);

		break;
	case TERM_VARIABLE_FLOAT_ARRAY:
		if(flag == HELPER_FLAG_DETAIL){
			for(uint32_t cnt=0;cnt<(var->typeSize / sizeof(float));cnt++){
				f_buffer = ((float*)var->variable)[cnt];
				TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
				ttprintf("\033[37m| [%u] \033[32m%f", cnt , f_buffer);
				if(cnt<(var->typeSize / sizeof(float)-1)){
					ttprintf("\r\n");
				}
			}
		}else{
			TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
			ttprintf("\033[37m| \033[32mArray[%u]", (var->typeSize / sizeof(float)));
		}

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
				ttprintf("\033[37m| %.2f", var->min_float);
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
				ttprintf("\033[37m| %.2f", var->max_float);
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

    	if(argCount && args[0] != NULL){
    		if(strstr(currVar->name, args[0])){
    			print_var_helperfunc(handle, currVar, flag);
    		}
    	}else{
    		print_var_helperfunc(handle, currVar, flag);
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
		u_temp_buffer = strtoul(value, NULL, 10);

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
		i_temp_buffer = strtol(value, NULL, 10);

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

//uint8_t buffer[2048];

typedef struct _FlashHeader_ FlashHeader;
typedef struct _FlashFooter_ FlashFooter;
typedef struct _FlashVariable_ FlashVariable;

struct _FlashHeader_{
	uint32_t start;
	uint32_t num_entries;
	uint32_t size;
	uint32_t version;
	uint32_t revision;
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
    FlashVariable * nextVar;
} __attribute__((packed));



//uint8_t * address = buffer;

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

static uint8_t * find_last_active_header(uint8_t * address, uint32_t storage_size){
	//Find last active header
	FlashHeader * header_ptr;
	header_ptr = (FlashHeader*)address;
	uint32_t size_last=0;
	uint8_t * header_section = address;
	while(header_ptr->start == HEADER_START && header_section < address + storage_size){
		header_section += header_ptr->size;
		size_last = header_ptr->size;
		header_ptr = (FlashHeader*)header_section;
	}
	return header_section - size_last;
}

uint32_t validate(FlashHeader * header){

	FlashVariable * FlashVar = (FlashVariable*)(header + 1);
	FlashVariable * currFlashVar = FlashVar;
	FlashFooter * footer = (FlashFooter *)((uint8_t*)header + header->size - sizeof(FlashFooter));

	uint32_t crc = TTERM_fnv1a_init();
	crc = TTERM_fnv1a_process_data(crc, header, sizeof(FlashHeader));

	//Checking CRC
	for(uint32_t currPos=0;currPos < header->num_entries; currPos++){
		crc = TTERM_fnv1a_process_data(crc, currFlashVar, sizeof(FlashVariable));
		currFlashVar = currFlashVar->nextVar;
	}

	currFlashVar = FlashVar;
	for(uint32_t currPos=0;currPos < header->num_entries; currPos++){
		crc = TTERM_fnv1a_process_data(crc, currFlashVar->name, currFlashVar->nameLength+1);
		crc = TTERM_fnv1a_process_data(crc, currFlashVar->variable, currFlashVar->typeSize);

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

	//Determine how much memory is needed
	uint32_t n_bytes = sizeof(FlashHeader) + (sizeof(FlashVariable) * head->nameLength) + sizeof(FlashFooter);
	for(;currPos < head->nameLength; currPos++){
		n_bytes += (currVar->nameLength+1);
		n_bytes += currVar->typeSize;
		currVar = currVar->nextVar;
	}

	if(n_bytes > storage_size){
		ttprintf("Memory overflow by %u bytes\r\n", n_bytes - storage_size);
		return TERM_CMD_EXIT_SUCCESS;
	}

	//Find last active header and increment revision
	FlashHeader * last_header = (FlashHeader*)find_last_active_header(address, storage_size);
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
		currData += nameLengthWNull;
		currData += currVar->typeSize;

		var_section++;
		currVar = currVar->nextVar;
	}

	//Write data
	currPos = 0;
	currVar = head->nextVar;
	currData = data_section;
	for(;currPos < head->nameLength; currPos++){

		uint32_t nameLengthWNull = currVar->nameLength+1;

		written += var->nvm_write(currData, (uint8_t*)currVar->name, nameLengthWNull);
		footer.crc = TTERM_fnv1a_process_data(footer.crc, currVar->name, nameLengthWNull);
		currData += nameLengthWNull;

		written += var->nvm_write(currData, currVar->variable, currVar->typeSize);
		footer.crc = TTERM_fnv1a_process_data(footer.crc, currVar->variable, currVar->typeSize);
		currData += currVar->typeSize;

		currVar = currVar->nextVar;
	}


	footer.end = FOOTER_END;

	footer.crc = TTERM_fnv1a_process_data(footer.crc, &footer, sizeof(FlashFooter)-sizeof(uint32_t));  //Dont CRC the crc field
	written += var->nvm_end_write(currData, &footer, sizeof(FlashFooter));

	ttprintf("Written: %u\r\n", written);

	uint32_t crc = validate((FlashHeader*)header_section);

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

	uint8_t * address = handle->varHandle->nvm_address;
	uint32_t storage_size = handle->varHandle->nvm_size;

	FlashHeader * header = (FlashHeader*)find_last_active_header(address, storage_size);

	if(header->start != HEADER_START || header->version != HEADER_VERSION){
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

	uint32_t crc = validate(header);

	if(footer->crc != crc){
		ttprintf("CRC mismatch Flash: %08x Loaded: %08x\r\n", footer->crc, crc);
		return TERM_CMD_EXIT_SUCCESS;
	}

	currPos = 0;
	currFlashVar = FlashVar;
	print_var_header_update(handle);

	handle->varHandle->nvm_revision = header->revision;
	for(;currPos < header->num_entries; currPos++){
		if(argCount){
			if(strcmp(currFlashVar->name, args[0]) != 0){
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
			if(strcmp(currFlashVar->name, currVar->name)==0 && currFlashVar->type == currVar->type && currFlashVar->typeSize == currVar->typeSize && (currVar->rw & VAR_ACCESS_W)){
				if(memcmp(currVar->variable, currFlashVar->variable, currVar->typeSize) != 0){
					memcpy(currVar->variable, currFlashVar->variable, currVar->typeSize);
					ttprintf("Updated value from flash\r\n");
				}else{
					ttprintf("-\r\n");
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

