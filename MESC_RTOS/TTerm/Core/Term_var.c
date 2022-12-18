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

#include "TTerm/Core/include/Term_var.h"
#include "TTerm/Core/include/TTerm.h"

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#define HEADER_START 	0xDEADBEEF
#define FOOTER_END   	0xDEADC0DE
#define HEADER_VERSION	0x00000001

char toLower(char c){
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

TermVariableDescriptor * TERM_addVarUnsigned(void* variable, uint16_t typeSize, uint32_t min, uint32_t max, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = min;
    newVAR->max_unsigned = max;
    newVAR->type = TERM_VARIABLE_UINT;
    newVAR->typeSize = typeSize;
    newVAR->name = name;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarSigned(void* variable, uint16_t typeSize, int32_t min, int32_t max, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_signed = min;
    newVAR->max_signed = max;
    newVAR->type = TERM_VARIABLE_INT;
    newVAR->typeSize = typeSize;
    newVAR->name = name;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarFloat(void* variable, float min, float max, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_float = min;
    newVAR->max_float = max;
    newVAR->type = TERM_VARIABLE_FLOAT;
    newVAR->typeSize = sizeof(float);
    newVAR->name = name;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarString(void* variable, uint16_t typeSize, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = 0;
    newVAR->max_unsigned = 0;
    newVAR->type = TERM_VARIABLE_STRING;
    newVAR->typeSize = typeSize;
    newVAR->name = name;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarChar(void* variable, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = 0;
    newVAR->max_unsigned = 0;
    newVAR->type = TERM_VARIABLE_CHAR;
    newVAR->typeSize = 1;
    newVAR->name = name;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarBool(void* variable, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = 0;
    newVAR->max_unsigned = 1;
    newVAR->type = TERM_VARIABLE_BOOL;
    newVAR->typeSize = sizeof(bool);
    newVAR->name = name;
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
				buff[commandsFound] = curr->name;
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

    //TODO use a reasonable size here
    handle->autocompleteBuffer = pvPortMalloc(handle->varListHead->nameLength * sizeof(char *));
    handle->currAutocompleteCount = 0;

    handle->autocompleteBufferLength = handle->varListHead->nameLength;
    handle->autocompleteBufferLength = TERM_doVarListAC(handle->varListHead, buff, len, handle->autocompleteBuffer);

    vPortFree(buff);
    return handle->autocompleteBufferLength;
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

static void print_var_helperfunc(TERMINAL_HANDLE * handle, TermVariableDescriptor * var, bool flash ){

    uint8_t current_parameter;
    uint32_t u_temp_buffer=0;
    int32_t i_temp_buffer=0;


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
	switch (var->type){
		case TERM_VARIABLE_UINT:
			ttprintf("\033[37m| %u", var->min_unsigned);
			break;
		case TERM_VARIABLE_INT:
			ttprintf("\033[37m| %i", var->min_signed);
			break;
		case TERM_VARIABLE_FLOAT:
			ttprintf("\033[37m| %.2f", var->min_float);
			break;
		case TERM_VARIABLE_BOOL:
			ttprintf("\033[37m| false");
			break;
		default:
			//ttprintf("\033[37m| -");
			break;
	}

	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_D);
	switch (var->type){
		case TERM_VARIABLE_UINT:
			ttprintf("\033[37m| %u", var->max_unsigned);
			break;
		case TERM_VARIABLE_INT:
			ttprintf("\033[37m| %i", var->max_signed);
			break;
		case TERM_VARIABLE_FLOAT:
			ttprintf("\033[37m| %.2f", var->max_float);
			break;
		case TERM_VARIABLE_BOOL:
			ttprintf("\033[37m| true");
			break;
		default:
			//ttprintf("\033[37m| -");
			break;
	}
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_E);
	if(flash==false){
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

	uint8_t currPos = 0;

	print_var_header(handle);

	TermVariableDescriptor * currVar = handle->varListHead->nextVar;
    for(;currPos < handle->varListHead->nameLength; currPos++){

    	if(argCount && args[0] != NULL){
    		if(strstr(currVar->name, args[0])){
    			print_var_helperfunc(handle, currVar, false);
    		}
    	}else{
    		print_var_helperfunc(handle, currVar, false);
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

static void set_value(TermVariableDescriptor * var, char* value){

    uint32_t u_temp_buffer=0;
    int32_t i_temp_buffer=0;
    uint32_t len = strlen(value);
    char* mul = NULL;

	switch (var->type){
	case TERM_VARIABLE_UINT:
		u_temp_buffer = strtoul(value, NULL, 10);
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
		*(float*)var->variable = strtof(value, &mul);
		if(mul != NULL){
			*(float*)var->variable *= getMul(*mul);
		}
		break;
	case TERM_VARIABLE_CHAR:
		*(char*)var->variable = value[0];
		break;
	case TERM_VARIABLE_STRING:
		strncpy((char*)var->variable, value, var->typeSize);
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

}

uint8_t CMD_varSet(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	uint32_t currPos = 0;
	if(argCount<2){
		ttprintf("Usage: set [name] [value]");
		return TERM_CMD_EXIT_SUCCESS;
	}

	TermVariableDescriptor * currVar = handle->varListHead->nextVar;
    for(;currPos < handle->varListHead->nameLength; currPos++){
    	if(strcmp(args[0], currVar->name)==0){
    		set_value(currVar, args[1]);
    		print_var_header(handle);
    		print_var_helperfunc(handle, currVar, false);
    		return TERM_CMD_EXIT_SUCCESS;
    	}

        currVar = currVar->nextVar;
    }


	return TERM_CMD_EXIT_SUCCESS;
}

uint8_t buffer[2048];

typedef struct _FlashHeader_ FlashHeader;
typedef struct _FlashFooter_ FlashFooter;

struct _FlashHeader_{
	uint32_t start;
	uint32_t num_entries;
	uint32_t size;
	uint32_t version;
} __attribute__((packed));

struct _FlashFooter_{
	uint32_t crc;
	uint32_t end;
} __attribute__((packed));


uint8_t * address = buffer;

volatile TermVariableDescriptor temp;

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

uint8_t CMD_varSave(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	FlashHeader header;
	FlashFooter footer;

	uint32_t currPos = 0;
	TermVariableDescriptor * currVar = handle->varListHead->nextVar;

	uint32_t storage_size = sizeof(buffer);
	uint8_t * header_section = address;


	//Determine how much memory is needed
	uint32_t n_bytes = sizeof(FlashHeader) + (sizeof(TermVariableDescriptor) * handle->varListHead->nameLength) + sizeof(FlashFooter);
	for(;currPos < handle->varListHead->nameLength; currPos++){
		n_bytes += (currVar->nameLength+1);
		n_bytes += currVar->typeSize;
		currVar = currVar->nextVar;
	}

	if(n_bytes > storage_size){
		ttprintf("Memory overflow by %u bytes\r\n", n_bytes - storage_size);
		return TERM_CMD_EXIT_SUCCESS;
	}

	header_section = find_next_free_memory(address, storage_size);

	//Check if new dataset fits into memory
	if(header_section + n_bytes > address + storage_size){
		header_section = address;
		ttprintf("Memory full, erasing flash...\r\n");
		memset(address,0, storage_size);
	}

	TermVariableDescriptor * var_section 	 = (TermVariableDescriptor *)(header_section + sizeof(FlashHeader));
	uint8_t * data_section 	 = header_section + sizeof(FlashHeader) + (sizeof(TermVariableDescriptor) * handle->varListHead->nameLength);


	header.start = HEADER_START;
	header.num_entries = handle->varListHead->nameLength;
	header.version = HEADER_VERSION;
	header.size = n_bytes;

	memcpy(header_section, &header, sizeof(header));

	currPos = 0;
	currVar = handle->varListHead->nextVar;
	for(;currPos < handle->varListHead->nameLength; currPos++){

		memcpy(&temp, currVar, sizeof(TermVariableDescriptor));

		uint32_t nameLengthWNull = currVar->nameLength+1;

		temp.name = data_section;
		temp.variableDescription = NULL;
		temp.variable = data_section + nameLengthWNull;

		if(currVar->nextVar){
			temp.nextVar = var_section + 1;
		}else{
			temp.nextVar = 0;
		}

		memcpy(var_section,&temp,sizeof(TermVariableDescriptor));

		memcpy(data_section, currVar->name, nameLengthWNull);
		data_section += nameLengthWNull;

		memcpy(data_section, currVar->variable, currVar->typeSize);
		data_section += currVar->typeSize;

		var_section++;
		currVar = currVar->nextVar;
	}


	footer.end = FOOTER_END;
	footer.crc = 0x00;

	memcpy(data_section, &footer, sizeof(FlashFooter));

	ttprintf("Saved %u variables in %u bytes\r\n", handle->varListHead->nameLength, n_bytes);

	return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_varLoad(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	uint32_t storage_size = sizeof(buffer);

	volatile FlashHeader * header = (FlashHeader*)find_last_active_header(address, storage_size);

	if(header->start != HEADER_START || header->version != HEADER_VERSION){
		ttprintf("No dataset found\r\n");
		return TERM_CMD_EXIT_SUCCESS;
	}
	FlashFooter * footer = (uint8_t*)header + header->size - sizeof(FlashFooter);
	if(footer->end != FOOTER_END){
		ttprintf("Unexpected footer\r\n");
		return TERM_CMD_EXIT_SUCCESS;
	}

	uint32_t currPos = 0;
	TermVariableDescriptor * flashVar = (TermVariableDescriptor*)(header + 1);
	print_var_header_update(handle);
	for(;currPos < header->num_entries; currPos++){
		if(argCount){
			if(strcmp(flashVar->name, args[0]) != 0){
				flashVar = flashVar->nextVar;
				continue;
			}
		}

		print_var_helperfunc(handle, flashVar, true);


		uint32_t currUpdatePos = 0;
		bool found_var=false;
		TermVariableDescriptor * currVar = handle->varListHead->nextVar;
		for(;currUpdatePos < handle->varListHead->nameLength; currUpdatePos++){
			if(strcmp(flashVar->name, currVar->name)==0 && flashVar->type == currVar->type && flashVar->typeSize == currVar->typeSize){
				if(memcmp(currVar->variable, flashVar->variable, currVar->typeSize) != 0){
					memcpy(currVar->variable, flashVar->variable, currVar->typeSize);
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

		flashVar = flashVar->nextVar;
	}


	return TERM_CMD_EXIT_SUCCESS;

}

